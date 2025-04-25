#include <xc.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "constants.h"
#include "interrupts.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "uart.h"
#include "globalTimers.h"
#include "fastPID.h"

// Global variables for the rotation counts.
// QEI1 is used for the right encoder and QEI2 for the left encoder.
int32_t rotationCount1; // QEI1 (ENCODER_LEFT)
int32_t rotationCount2; // QEI2 (ENCODER_RIGHT)

//--------------------------------------------------------------------------
// Global volatile variables holding the most recent velocity estimates.
// We assume the enumeration for MotorEncoder_t maps ENCODER_LEFT -> 0
// and ENCODER_RIGHT -> 1.
//--------------------------------------------------------------------------
volatile float encoderPosEstRad[2]      = { 0.0f, 0.0f };
volatile float encoderVelEstRad[2]      = { 0.0f, 0.0f };
volatile float encoderVelIntegrator[2]  = { 0.0f, 0.0f };

//volatile float currentVelocityDegPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityRadPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityMmPerSec[2] = {0.0f, 0.0f};
volatile int32_t currentVelocityCounts[2]  = {0, 0};

// Global storage for last encoder positions and the time at which they
// were last updated. These are used only by the timer-driven update.
volatile int32_t lastEncoderPosition[2] = {0, 0};
volatile uint64_t lastVelocityUpdateTime = 0;

// PI gains
#define ENC_PID_KP  100.5f
#define ENC_PID_KI  3950.0f

#define ENC_MIN_VEL 0.001f
/*-------------------------------------------------------------------------
  Utility Function: readEncoderCount
  Atomically reads the encoder count by combining the hardware position counter 
  with the software-maintained rotation counter.
-------------------------------------------------------------------------*/
static inline int32_t readEncoderCount(MotorEncoder_t encoder)
{
    int32_t count;
    _NSTDIS = 1;
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
    _NSTDIS = 0;
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
    return currentEncoderPosition * TICKS_TO_RAD;
}

/*-------------------------------------------------------------------------
  Function: getPositionInRad
  Converts the encoder count into a position in degrees.
-------------------------------------------------------------------------*/
float getEncoderPositionDeg(MotorEncoder_t encoder)
{
    int32_t currentEncoderPosition = readEncoderCount(encoder);
    return currentEncoderPosition * TICKS_TO_DEG;
}

//*****************************************************************************
// Called periodically (e.g., via a timer interrupt) to:
//   - Determine the change in encoder counts since its last call
//   - Compute the angular velocity (in both rad/s and deg/s)
//   - Update the global timestamps and positions.
// Time is measured in microseconds using getTimeInUs().
//*****************************************************************************
void updateEncoderVelocities(void)
{
    // Sample the current time and read both encoders together.
    uint64_t currentTimeUs = getTimeInUs();
    int32_t leftPos = readEncoderCount(ENCODER_LEFT);
    int32_t rightPos = readEncoderCount(ENCODER_RIGHT);
    
    uint64_t deltaTime = currentTimeUs - lastVelocityUpdateTime;
    float dtSec = deltaTime * 1e-6f;
    lastVelocityUpdateTime = currentTimeUs;
    
    // --- Left encoder PI tracking loop ---
    {
        // measured position in radians
        float measPos = leftPos * TICKS_TO_RAD;
        // predict next pos: x_est += v_est * dt
        encoderPosEstRad[ENCODER_LEFT] += encoderVelEstRad[ENCODER_LEFT] * dtSec;
        // position error
        float err = measPos - encoderPosEstRad[ENCODER_LEFT];
        // integrator update
        encoderVelIntegrator[ENCODER_LEFT] += err * ENC_PID_KI * dtSec;
        // velocity estimate: v = Kp*err + Ki_integral
        encoderVelEstRad[ENCODER_LEFT] = err * ENC_PID_KP + encoderVelIntegrator[ENCODER_LEFT];
        // clamp
        currentVelocityRadPerSec[ENCODER_LEFT] = encoderVelEstRad[ENCODER_LEFT] < ENC_MIN_VEL ? 0.0f : encoderVelEstRad[ENCODER_LEFT];
        // publish
        currentVelocityMmPerSec[ENCODER_LEFT]  = encoderVelEstRad[ENCODER_LEFT] * WHEEL_RADIUS_MM;
        // still update lastEncoderPosition for rollover logic
        lastEncoderPosition[ENCODER_LEFT]      = leftPos;
            
        /*
        static uint16_t i = 0; 
        if (i % 5 == 0) { 
            char buf[100]; 
            snprintf(buf, sizeof(buf), "Left: meas %.4f, est_pos %.4f, err %.4f, ctl %.4f\r\n", measPos, encoderPosEstRad[ENCODER_LEFT], (err), encoderVelEstRad[ENCODER_LEFT]); 
            putsUART1Str(buf); 
        } 
        i++;
        */
    }

    // --- Right encoder PI tracking loop ---
    {
        float measPos = rightPos * TICKS_TO_RAD;
        encoderPosEstRad[ENCODER_RIGHT] += encoderVelEstRad[ENCODER_RIGHT] * dtSec;
        float err = measPos - encoderPosEstRad[ENCODER_RIGHT];
        encoderVelIntegrator[ENCODER_RIGHT] += err * ENC_PID_KI * dtSec;
        encoderVelEstRad[ENCODER_RIGHT] = err * ENC_PID_KP + encoderVelIntegrator[ENCODER_RIGHT];
        currentVelocityRadPerSec[ENCODER_RIGHT] = encoderVelEstRad[ENCODER_RIGHT] < ENC_MIN_VEL ? 0.0f : encoderVelEstRad[ENCODER_RIGHT];
        currentVelocityMmPerSec[ENCODER_RIGHT]  = encoderVelEstRad[ENCODER_RIGHT] * WHEEL_RADIUS_MM;
        lastEncoderPosition[ENCODER_RIGHT]      = rightPos;
    }
}

int32_t getEncoderVelocityCountsPerSample(MotorEncoder_t encoder)
{
    return currentVelocityCounts[encoder];
}

float getEncoderVelocityRadPerSec(MotorEncoder_t encoder)
{
    return currentVelocityRadPerSec[encoder];
}

float getEncoderVelocityDegPerSec(MotorEncoder_t encoder)
{
    return currentVelocityRadPerSec[encoder] * RAD2DEG;
    //return currentVelocityDegPerSec[encoder];
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
    float yawRateRadPerSec = (vLeft - vRight) / WHEEL_BASE_MM;

    return yawRateRadPerSec;
}

float getEncoderLinearVelocityMmPerSec(void)
{
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    // Compute forward (translational) velocity.
    return (vLeft + vRight) * 0.5f;
}

void getEncoderLinearVelocityAndYawRate(float* linearVelocityMmPerSec, float* yawRateRadPerSec) {
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    *linearVelocityMmPerSec = (vLeft + vRight) * 0.5f;
    
    // Yaw rate (rad/s) using differential drive model.
    *yawRateRadPerSec = (vRight - vLeft) / WHEEL_BASE_MM;
}