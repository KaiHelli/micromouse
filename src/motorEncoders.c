/* Source code inspired by:
 *
 * Encoder Velocity PLL State Estimator: Odrive Robotics
 * - https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224
 * - https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/encoder.cpp
 */ 

#include <xc.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "atomic.h"
#include "constants.h"
#include "interrupts.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "uart.h"
#include "globalTimers.h"
#include "fastPID.h"

// Global variables for the rotation counts.
// QEI1 is used for the left encoder and QEI2 for the right encoder.
volatile int32_t rotationCount1; // QEI1 (ENCODER_LEFT)
volatile int32_t rotationCount2; // QEI2 (ENCODER_RIGHT)

//--------------------------------------------------------------------------
// Global volatile variables holding the most recent velocity estimates.
// We assume the enumeration for MotorEncoder_t maps ENCODER_LEFT -> 0
// and ENCODER_RIGHT -> 1.
//--------------------------------------------------------------------------
volatile float encoderPosEst[2]         = { 0.0f, 0.0f };
volatile float encoderVelEst[2]         = { 0.0f, 0.0f };

//volatile float currentVelocityDegPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityRadPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityMmPerSec[2] = {0.0f, 0.0f};

// Global storage for last encoder positions and the time at which they
// were last updated. These are used only by the timer-driven update.
volatile uint64_t lastVelocityUpdateTime = 0;

volatile bool encVelInit = false;

// PI gains
#define ENC_PLL_KP  2.0f * 50.0f
#define ENC_PLL_KI  (0.25f * ENC_PLL_KP * ENC_PLL_KP)

#define ENC_MIN_VEL 0.001f
/*-------------------------------------------------------------------------
  Utility Function: readEncoderCount
  Atomically reads the encoder count by combining the hardware position counter 
  with the software-maintained rotation counter.
-------------------------------------------------------------------------*/
static inline int32_t readEncoderCount(MotorEncoder_t encoder)
{
    int32_t count;
    
    uint16_t state = _atomic_enter();
    
    switch(encoder)
    {
        case ENCODER_LEFT:
            count = rotationCount1 + POSCNT;
            break;
        case ENCODER_RIGHT:
            count = rotationCount2 + POS2CNT;
            break;
        default:
            count = 0;
            break;
    }
    
    _atomic_leave(state);
    
    return count;
}

void initMotorEncoders() {
    initQEI(ENCODER_LEFT, 0);
    initQEI(ENCODER_RIGHT, 0);
}

/*-------------------------------------------------------------------------
  Function: initQEI
  Initializes either QEI1 (for ENCODER_RIGHT) or QEI2 (for ENCODER_LEFT) 
  based on the encoder parameter.
-------------------------------------------------------------------------*/
void initQEI(MotorEncoder_t encoder, uint16_t startPos)
{
    switch(encoder)
    {
        case ENCODER_LEFT:
            QEI1CONbits.QEISIDL = 1;         // Disable module in idle mode
            QEI1CONbits.QEIM = 0b111;        // 4x mode with reset by match
            QEI1CONbits.SWPAB = 1;           // Swap Phase A and B
            QEI1CONbits.PCDOUT = 0;          // disable position counter direction pin 
            QEI1CONbits.TQGATE = 0;          // timer gated time acc disabled
            QEI1CONbits.POSRES = 0;          // index does not reset position counter
            QEI1CONbits.TQCS = 0;            // internal clock source (Tcy))
            QEI1CONbits.UPDN_SRC = 0;        // direction of position counter determined using internal logic
            
            MAXCNT = 0xffff;
            POSCNT = startPos;
            rotationCount1 = 0;
            
            IFS3bits.QEI1IF = 0;             // clear interrupt flag 
            IEC3bits.QEI1IE = 1;             // enable interrupt
            IPC14bits.QEI1IP = IP_QEI;
            break;
        case ENCODER_RIGHT:
            QEI2CONbits.QEISIDL = 1;         // Disable module in idle mode
            QEI2CONbits.QEIM = 0b111;        // 4x mode with reset by match
            QEI2CONbits.SWPAB = 1;           // Swap Phase A and B
            QEI2CONbits.PCDOUT = 0;          // disable position counter direction pin 
            QEI2CONbits.TQGATE = 0;          // timer gated time acc disabled
            QEI2CONbits.POSRES = 0;          // index does not reset position counter
            QEI2CONbits.TQCS = 0;            // internal clock source (Tcy))
            QEI2CONbits.UPDN_SRC = 0;        // direction of position counter determined using internal logic
            
            MAX2CNT = 0xffff;
            POS2CNT = startPos;
            rotationCount2 = 0;
            
            IFS4bits.QEI2IF = 0;             // clear interrupt flag 
            IEC4bits.QEI2IE = 1;             // enable interrupt
            IPC18bits.QEI2IP = IP_QEI;
            break;
        default:
            break;
    }
}

/*-------------------------------------------------------------------------
  Interrupt Service Routines for QEI roll-over/under detection
-------------------------------------------------------------------------*/
void __attribute__((__interrupt__, auto_psv)) _QEI1Interrupt(void)
{
    IFS3bits.QEI1IF = 0; 
    if (POSCNT < 32768) {
        rotationCount1 += ENC_MAX_VALUE;  // Positive roll-over
    } else {
        rotationCount1 -= ENC_MAX_VALUE;  // Negative roll-over
    }
}

void __attribute__((__interrupt__, auto_psv)) _QEI2Interrupt(void)
{
    IFS4bits.QEI2IF = 0; 
    if (POS2CNT < 32768) {
        rotationCount2 += ENC_MAX_VALUE;  // Positive roll-over
    } else {
        rotationCount2 -= ENC_MAX_VALUE;  // Negative roll-over
    }
}

/*-------------------------------------------------------------------------
  Function: getPositionInCounts
  Returns the current encoder position (including any rotation adjustments).
-------------------------------------------------------------------------*/
int32_t getEncoderPositionCounts(MotorEncoder_t encoder)
{
    return readEncoderCount(encoder);
}

/*-------------------------------------------------------------------------
  Function: getPositionInRad
  Converts the encoder count into a position in radians.
-------------------------------------------------------------------------*/
float getEncoderPositionRad(MotorEncoder_t encoder)
{
    int32_t currentEncoderPosition = readEncoderCount(encoder);
    return currentEncoderPosition * ENC_TICKS_TO_RAD;
}

/*-------------------------------------------------------------------------
  Function: getPositionInRad
  Converts the encoder count into a position in degrees.
-------------------------------------------------------------------------*/
float getEncoderPositionDeg(MotorEncoder_t encoder)
{
    int32_t currentEncoderPosition = readEncoderCount(encoder);
    return currentEncoderPosition * ENC_TICKS_TO_DEG;
}

//*****************************************************************************
// Called periodically (e.g., via a timer interrupt) to:
//   - Determine the change in encoder counts since its last call
//   - Compute the angular velocity (in both rad/s and deg/s)
//   - Update the global timestamps and positions.
// Time is measured in microseconds using getTimeInUs().
//*****************************************************************************
int16_t updateEncoderVelocities(void)
{
    // Sample the current time and read both encoders together.
    uint64_t currentTimeUs = getTimeInUs();
    
    int32_t leftPos = readEncoderCount(ENCODER_LEFT);
    int32_t rightPos = readEncoderCount(ENCODER_RIGHT);
    
    uint64_t deltaTime = currentTimeUs - lastVelocityUpdateTime;
    float dtSec = deltaTime * 1e-6f;
    lastVelocityUpdateTime = currentTimeUs;
    
    if(!encVelInit) {
        encoderPosEst[ENCODER_LEFT] = leftPos;
        encoderPosEst[ENCODER_RIGHT] = rightPos;
        encVelInit = true;
        return 1;
    }
    
    // --- Left encoder PI tracking loop ---
    {
        // measured position in radians
        int32_t measPos = leftPos;
        // predict next pos: x_est += v_est * dt
        encoderPosEst[ENCODER_LEFT] += encoderVelEst[ENCODER_LEFT] * dtSec;
        
        // position error
        float deltaPos = (float) (measPos - (int32_t) floorf(encoderPosEst[ENCODER_LEFT]));
        
        // update
        encoderPosEst[ENCODER_LEFT] += ENC_PLL_KP * deltaPos * dtSec;
        encoderVelEst[ENCODER_LEFT] += ENC_PLL_KI * deltaPos * dtSec;
        
        if (fabs(encoderVelEst[ENCODER_LEFT]) < 0.5f * dtSec * ENC_PLL_KI) {
            encoderVelEst[ENCODER_LEFT] = 0.0f;
        }
        
        currentVelocityRadPerSec[ENCODER_LEFT] = encoderVelEst[ENCODER_LEFT] * ENC_TICKS_TO_RAD;
        currentVelocityMmPerSec[ENCODER_LEFT]  = encoderVelEst[ENCODER_LEFT] * ENC_DIST_PER_TICK_MM;
    }
    // --- Right encoder PI tracking loop ---
    {
        int32_t measPos = rightPos;
        encoderPosEst[ENCODER_RIGHT] += encoderVelEst[ENCODER_RIGHT] * dtSec;
        float deltaPos = (float) (measPos - (int32_t) floorf(encoderPosEst[ENCODER_RIGHT]));
        
        encoderPosEst[ENCODER_RIGHT] += ENC_PLL_KP * deltaPos * dtSec;
        encoderVelEst[ENCODER_RIGHT] += ENC_PLL_KI * deltaPos * dtSec;
        
        if (fabs(encoderVelEst[ENCODER_RIGHT]) < 0.5f * dtSec * ENC_PLL_KI) {
            encoderVelEst[ENCODER_RIGHT] = 0.0f;
        }
        
        currentVelocityRadPerSec[ENCODER_RIGHT] = encoderVelEst[ENCODER_RIGHT] * ENC_TICKS_TO_RAD;
        currentVelocityMmPerSec[ENCODER_RIGHT]  = encoderVelEst[ENCODER_RIGHT] * ENC_DIST_PER_TICK_MM;
    }
    
    return 1;
}

int16_t updateEncoderVelocitiesNaive(float hz)
{
    static int32_t lastLeftPos;
    static int32_t lastRightPos;
    
    int32_t leftPos = readEncoderCount(ENCODER_LEFT);
    int32_t rightPos = readEncoderCount(ENCODER_RIGHT);
    
    int32_t dLeftPos = leftPos - lastLeftPos;
    int32_t dRightPos = rightPos - lastRightPos;
    
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    
    if(!encVelInit) {
        encVelInit = true;
        return 1;
    }
    
    currentVelocityMmPerSec[ENCODER_LEFT]  = (float) dLeftPos  * ENC_DIST_PER_TICK_MM * hz;
    currentVelocityMmPerSec[ENCODER_RIGHT] = (float) dRightPos * ENC_DIST_PER_TICK_MM * hz;
    
    return 1;
}

float getEncoderVelocityRadPerSec(MotorEncoder_t encoder)
{
    return currentVelocityRadPerSec[encoder];
}

float getEncoderVelocityDegPerSec(MotorEncoder_t encoder)
{
    return currentVelocityRadPerSec[encoder] * RAD2DEG;
}

float getEncoderVelocityMmPerSec(MotorEncoder_t encoder)
{
    return currentVelocityMmPerSec[encoder];
}

float getEncoderYawRateRadPerSec(void)
{
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    // Yaw rate (rad/s) using differential drive model.
    float yawRateRadPerSec = (vLeft - vRight) / MOUSE_WHEEL_SEPARATION_MM;

    return yawRateRadPerSec;
}

float getEncoderLinearVelocityMmPerSec(void)
{
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    // Compute forward (translational) velocity.
    return (vLeft + vRight) * 0.5f;
}

float getEncoderAverageDistanceMm(void)
{
    return ((float) getEncoderPositionCounts(ENCODER_LEFT) * ENC_DIST_PER_TICK_MM + (float) getEncoderPositionCounts(ENCODER_RIGHT) * ENC_DIST_PER_TICK_MM) * 0.5f;
}

float getEncoderAverageDistanceUm(void)
{
    return ((float) getEncoderPositionCounts(ENCODER_LEFT) * ENC_DIST_PER_TICK_UM + (float) getEncoderPositionCounts(ENCODER_RIGHT) * ENC_DIST_PER_TICK_UM) * 0.5f;
}

void getEncoderLinearVelocityAndYawRate(float* linearVelocityMmPerSec, float* yawRateRadPerSec) {
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    *linearVelocityMmPerSec = (vLeft + vRight) * 0.5f;
    
    // Yaw rate (rad/s) using differential drive model.
    *yawRateRadPerSec = (vRight - vLeft) / MOUSE_WHEEL_SEPARATION_MM;
}