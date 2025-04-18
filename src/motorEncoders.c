#include <xc.h>
#include <math.h>
#include <stdint.h>

#include "constants.h"
#include "interrupts.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "uart.h"
#include "globalTimers.h"

// Global variables for the rotation counts.
// QEI1 is used for the right encoder and QEI2 for the left encoder.
int32_t rotationCount1; // QEI1 (ENCODER_LEFT)
int32_t rotationCount2; // QEI2 (ENCODER_RIGHT)

//--------------------------------------------------------------------------
// Global volatile variables holding the most recent velocity estimates.
// We assume the enumeration for MotorEncoder_t maps ENCODER_LEFT -> 0
// and ENCODER_RIGHT -> 1.
//--------------------------------------------------------------------------
//volatile float currentVelocityDegPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityRadPerSec[2] = {0.0f, 0.0f};
volatile float currentVelocityMmPerSec[2] = {0.0f, 0.0f};
volatile int32_t currentVelocityCounts[2]  = {0, 0};

// Global storage for last encoder positions and the time at which they
// were last updated. These are used only by the timer-driven update.
volatile int32_t lastEncoderPosition[2] = {0, 0};
volatile uint64_t lastVelocityUpdateTime = 0;

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
// Timer-based Update Function: updateEncoderVelocities
//
// Called periodically (e.g., via a timer interrupt) to:
//   - Determine the change in encoder counts since its last call
//   - Compute the angular velocity (in both rad/s and deg/s)
//   - Store the delta counts for legacy getVelocityInCountsPerSample()
//   - Update the global timestamps and positions.
// Time is measured in microseconds using getTimeInUs().
//*****************************************************************************
void updateEncoderVelocities(void)
{
    // Sample the current time and read both encoders together.
    uint64_t currentTimeUs = getTimeInUs();
    int32_t leftPos = readEncoderCount(ENCODER_LEFT);
    int32_t rightPos = readEncoderCount(ENCODER_RIGHT);
    
    // Precompute conversion constants.
    const float microsecToSecFactor = 1e6f;  // Converts microseconds to seconds when used as (1e6 / deltaTime)

    uint64_t deltaTime = currentTimeUs - lastVelocityUpdateTime;
    float invTime = (microsecToSecFactor / deltaTime);
    
    lastVelocityUpdateTime = currentTimeUs;
    
    // Use an exponential moving average of past measurements for smoothing
    const float alpha = 0.5f;
    
    // --- Update for ENCODER_LEFT ---
    {
        int32_t deltaCounts = leftPos - lastEncoderPosition[ENCODER_LEFT];
        
        // Compute the combined multiplication once.
        float deltaCountInvTime = deltaCounts * invTime;
        
        float newVelocityRad = deltaCountInvTime * TICKS_TO_RAD;
        //float newVelocityDeg = deltaCountInvTime * TICKS_TO_DEG;
        
        currentVelocityCounts[ENCODER_LEFT] = deltaCounts;
        currentVelocityRadPerSec[ENCODER_LEFT] = alpha * newVelocityRad + (1 - alpha) * currentVelocityRadPerSec[ENCODER_LEFT];
        currentVelocityMmPerSec[ENCODER_LEFT] = currentVelocityRadPerSec[ENCODER_LEFT] * WHEEL_RADIUS_MM;
        //currentVelocityDegPerSec[ENCODER_LEFT] = alpha * newVelocityDeg + (1 - alpha) * currentVelocityDegPerSec[ENCODER_LEFT];
        
        lastEncoderPosition[ENCODER_LEFT] = leftPos;
    }

    // --- Update for ENCODER_RIGHT ---
    {
        int32_t deltaCounts = rightPos - lastEncoderPosition[ENCODER_RIGHT];
        
        float deltaCountInvTime = deltaCounts * invTime;
        
        float newVelocityRad = deltaCountInvTime * TICKS_TO_RAD;
        //float newVelocityDeg = deltaCountInvTime * TICKS_TO_DEG;
        
        currentVelocityCounts[ENCODER_RIGHT] = deltaCounts;
        currentVelocityRadPerSec[ENCODER_RIGHT] = alpha * newVelocityRad + (1 - alpha) * currentVelocityRadPerSec[ENCODER_RIGHT];
        currentVelocityMmPerSec[ENCODER_RIGHT] = currentVelocityRadPerSec[ENCODER_RIGHT] * WHEEL_RADIUS_MM;
        //currentVelocityDegPerSec[ENCODER_RIGHT] = alpha * newVelocityDeg + (1 - alpha) * currentVelocityDegPerSec[ENCODER_RIGHT];

        lastEncoderPosition[ENCODER_RIGHT] = rightPos;
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