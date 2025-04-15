#include <xc.h>
#include <math.h>
#include <stdint.h>

#include "interrupts.h"
#include "IOconfig.h"
#include "motorEncoders.h"
#include "uart.h"
#include "globalTimers.h"

// Global variables for the rotation counts.
// QEI1 is used for the right encoder and QEI2 for the left encoder.
int32_t rotationCount1; // QEI1 (ENCODER_RIGHT)
int32_t rotationCount2; // QEI2 (ENCODER_LEFT)

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
        case ENCODER_RIGHT:
            count = rotationCount1 + POSCNT;
            break;
        case ENCODER_LEFT:
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
        case ENCODER_RIGHT:
            QEI1CONbits.QEISIDL = 1;       // Disable module in idle mode
            QEI1CONbits.QEIM = 0b111;        // 4x mode with reset by match
            QEI1CONbits.SWPAB = 0;           // Do not swap Phase A and B
            QEI1CONbits.PCDOUT = 0;
            QEI1CONbits.TQGATE = 0;
            QEI1CONbits.POSRES = 0;
            QEI1CONbits.TQCS = 0;
            QEI1CONbits.UPDN_SRC = 0;
            
            MAXCNT = 0xffff;
            POSCNT = startPos;
            rotationCount1 = 0;
            
            IFS3bits.QEI1IF = 0;
            IEC3bits.QEI1IE = 1;
            IPC14bits.QEI1IP = IP_QEI;
            break;
        case ENCODER_LEFT:
            QEI2CONbits.QEISIDL = 1;       // Disable module in idle mode
            QEI2CONbits.QEIM = 0b111;        // 4x mode with reset by match
            QEI2CONbits.SWPAB = 0;           // Configure as needed for your hardware
            QEI2CONbits.PCDOUT = 0;
            QEI2CONbits.TQGATE = 0;
            QEI2CONbits.POSRES = 0;
            QEI2CONbits.TQCS = 0;
            QEI2CONbits.UPDN_SRC = 0;
            
            MAX2CNT = 0xffff;
            POS2CNT = startPos;
            rotationCount2 = 0;
            
            IFS4bits.QEI2IF = 0;
            IEC4bits.QEI2IE = 1;
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
int32_t getPositionInCounts(MotorEncoder_t encoder)
{
    return readEncoderCount(encoder);
}

/*-------------------------------------------------------------------------
  Function: getPositionInRad
  Converts the encoder count into a position in radians.
-------------------------------------------------------------------------*/
float getPositionInRad(MotorEncoder_t encoder)
{
    int32_t currentEncoderPosition = readEncoderCount(encoder);
    return (2.0f * M_PI * currentEncoderPosition) / TICKS_PER_REV;
}

/*-------------------------------------------------------------------------
  Function: getPositionInRad
  Converts the encoder count into a position in degrees.
-------------------------------------------------------------------------*/
float getPositionInDeg(MotorEncoder_t encoder)
{
    int32_t currentEncoderPosition = readEncoderCount(encoder);
    return (360.0f * currentEncoderPosition) / TICKS_PER_REV;
}

/*-------------------------------------------------------------------------
  Function: getVelocityInCountsPerSample
  Computes the change in encoder counts between calls.
-------------------------------------------------------------------------*/
int16_t getVelocityInCountsPerSample(MotorEncoder_t encoder)
{
    // Using a static array to store previous positions for each encoder.
    static int32_t oldPosition[2] = {0, 0};
    int32_t currentPosition = readEncoderCount(encoder);
    int16_t velocity = (int16_t)(currentPosition - oldPosition[encoder]);
    oldPosition[encoder] = currentPosition;
    return velocity;
}

/*-------------------------------------------------------------------------
  Function: getVelocityInRadPerSecond
  Computes the rotational velocity in radians per second.
-------------------------------------------------------------------------*/
float getVelocityInRadPerSecond(MotorEncoder_t encoder)
{
    // Static arrays to store timing and previous position for each encoder.
    static uint64_t lastGyroUpdateTime[2] = {0, 0};
    static int32_t oldPosition[2] = {0, 0};

    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs = currentTimeUs - lastGyroUpdateTime[encoder];
    lastGyroUpdateTime[encoder] = currentTimeUs;
    
    int32_t currentPosition = readEncoderCount(encoder);
    float velocity = 2.0f * M_PI * ((currentPosition - oldPosition[encoder]) * deltaUs * 1e-6f) / TICKS_PER_REV;
    oldPosition[encoder] = currentPosition;
    return velocity;
}

/*-------------------------------------------------------------------------
  Function: getVelocityInDegPerSecond
  Computes the rotational velocity in degrees per second.
-------------------------------------------------------------------------*/
float getVelocityInDegPerSecond(MotorEncoder_t encoder)
{
    // Static arrays to hold the previous update time and previous position for each encoder.
    static uint64_t lastUpdateTime[2] = {0, 0};
    static int32_t oldPosition[2] = {0, 0};

    // Get current time in microseconds.
    uint64_t currentTimeUs = getTimeInUs();
    // Calculate time elapsed since the last update in microseconds.
    uint64_t deltaUs = currentTimeUs - lastUpdateTime[encoder];
    // Update the last update time for the current encoder.
    lastUpdateTime[encoder] = currentTimeUs;

    // Read the current encoder position.
    int32_t currentPosition = readEncoderCount(encoder);
    // Calculate the velocity in degrees per second.
    // Multiply delta counts by the elapsed time (converted to seconds) then convert to degrees.
    float velocity = (360.0f * ((currentPosition - oldPosition[encoder]) * deltaUs * 1e-6f)) / TICKS_PER_REV;
    // Update the old position for the current encoder.
    oldPosition[encoder] = currentPosition;
    return velocity;
}

/*-------------------------------------------------------------------------
  Function: getEncoderYawDeg
  Computes an approximate yaw (in degrees) from the difference between
  the right and left encoder distances.
-------------------------------------------------------------------------*/
float getEncoderYawDeg(void)
{
    float rightDistance = readEncoderCount(ENCODER_RIGHT) * DIST_PER_TICK;
    float leftDistance  = readEncoderCount(ENCODER_LEFT) * DIST_PER_TICK;
    return (rightDistance - leftDistance) * RAD2DEG / WHEEL_BASE_MM;
}
