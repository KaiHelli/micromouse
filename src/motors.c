#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "motors.h"
#include "IOConfig.h"
#include "odometry.h"
#include "timers.h"
#include "fastPID.h"
#include "uart.h"
#include "motorEncoders.h"

#define WHEEL_PID_KP  0.12f
#define WHEEL_PID_KI  1.0f
#define WHEEL_PID_KD  0.0f
#define WHEEL_PID_KF  0.183f
//#define WHEEL_PID_BC  0.000f

static FastPid pidL, pidR;
static float motorSpeedLeft = 0;
static float motorSpeedRight = 0;

static int16_t wheelPidStep(void)
{
    // Don't control if motors are in standby
    // TODO: Reads latch... correct?
    if (!M_STDBY) {
        return 1;
    }
    
    float vLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float vRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    
    int16_t pLeft = fastPidStep(&pidL, (int16_t) (motorSpeedLeft),  (int16_t) (vLeft));
    int16_t pRight = fastPidStep(&pidR, (int16_t) (motorSpeedRight), (int16_t) (vRight));

    /*
    static uint16_t i = 0;
    if (i % 1 == 0) {
        char buf[100];
        //snprintf(buf, sizeof(buf), "Left: sp %d, is %d, ctl %d | Right: sp %d, is %d, ctl %d\r\n", (int16_t) (motorSpeedLeft * 10), (int16_t) (vLeft * 10), pLeft, (int16_t) (motorSpeedRight * 10), (int16_t) (vRight * 10), pRight);
        snprintf(buf, sizeof(buf), "%.2f, %.2f\r\n", motorSpeedLeft, motorSpeedRight);
        putsUART1Str(buf);
    }
    i++;
    */
    
    /*
    uint8_t buffer[20];
    size_t idx = 0;
    buffer[idx++] = FRAME_START_BYTE;
    
    float errLeft = motorSpeedLeft - vLeft;
    float errRight = motorSpeedRight - vRight;
    
    memcpy(&buffer[idx], &errLeft, sizeof(errLeft));
    idx += sizeof(errLeft);
    memcpy(&buffer[idx], &errRight, sizeof(errRight));
    idx += sizeof(errRight);
    
    memcpy(&buffer[idx], &pLeft, sizeof(pLeft));
    idx += sizeof(pLeft);
    memcpy(&buffer[idx], &pRight, sizeof(pRight));
    idx += sizeof(pRight);
    
    buffer[idx++] = FRAME_END_BYTE;
    putsUART1(buffer, idx);
    */
    
    
    setMotorPower(MOTOR_LEFT,  pLeft);
    setMotorPower(MOTOR_RIGHT, pRight);
    
    return 1;
}

void initMotorsState(Timer_t timer, float pid_hz) {
    // Set motors to standby on startup
    setMotorsStandbyState(true);
    
    fastPidInit(&pidL);  
    fastPidInit(&pidR);
    fastPidConfigure(&pidL, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_KF, pid_hz, 8, true);
    fastPidConfigure(&pidR, WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD, WHEEL_PID_KF, pid_hz, 8, true);
    fastPidSetOutputRange(&pidL, -100, 100);
    fastPidSetOutputRange(&pidR, -100, 100);
    //fastPidSetAntiWindup(&pidL, FASTPID_AW_BACKCALC, WHEEL_PID_BC);
    //fastPidSetAntiWindup(&pidR, FASTPID_AW_BACKCALC, WHEEL_PID_BC);
    fastPidSetAntiWindup(&pidL, FASTPID_AW_CLAMP, 0.0f);
    fastPidSetAntiWindup(&pidL, FASTPID_AW_CLAMP, 0.0f);
    
    if (fastPidHasConfigError(&pidL) || fastPidHasConfigError(&pidR)) {
        putsUART1Str("Failed to setup PIDs for motor control.\r\n");
        return;
    }
    
    registerTimerCallback(timer, wheelPidStep);
}

void setMotorSpeedLeft(int16_t mmPerSec)
{
    motorSpeedLeft = mmPerSec;
}

void setMotorSpeedRight(int16_t mmPerSec)
{
    motorSpeedRight = mmPerSec;
}

void setMotorsStandbyState(bool state) {
    M_STDBY = !state;
}

void toggleMotorsStandby(){
    M_STDBY = !M_STDBY;
}

void setMotorsState(MotorState_t state){
    switch (state) { 
        case MOTORS_BRAKE:
            MA_DIR1 = H;
            MA_DIR2 = H;
            MB_DIR1 = H;
            MB_DIR2 = H;
            break;
        case MOTORS_BACKWARD:
            MA_DIR1 = H;
            MA_DIR2 = L;
            MB_DIR1 = H;
            MB_DIR2 = L;
            break;
        case MOTORS_FORWARD:
            MA_DIR1 = L;
            MA_DIR2 = H;
            MB_DIR1 = L;
            MB_DIR2 = H;
            break;
        case MOTORS_ROTATE_LEFT:
            MA_DIR1 = H;
            MA_DIR2 = L;
            MB_DIR1 = L;
            MB_DIR2 = H;
            break;
        case MOTORS_ROTATE_RIGHT:
            MA_DIR1 = L;
            MA_DIR2 = H;
            MB_DIR1 = H;
            MB_DIR2 = L;
            break;
    }
}

void steerMotors(int8_t steering, uint8_t powerInPercent)
{
    // Clamp power to [0..100]
    if (powerInPercent > 100) {
        powerInPercent = 100;
    }

    // Clamp steering to [-100..100]
    if (steering < -100) {
        steering = -100;
    }
    if (steering > 100) {
        steering = 100;
    }

    // Compute ratio as integer in range [-50..50]
    int16_t ratio = 50 - (steering >= 0 ? steering : -steering);

    // Start with both motors at full requested power
    int16_t leftPercent  = powerInPercent;
    int16_t rightPercent = powerInPercent;

    // Scale only the appropriate side
    if (steering >= 0) {
        // Steer right => reduce right motor by ratio
        rightPercent = (rightPercent * ratio) / 50;
    } else {
        // Steer left => reduce left motor by ratio
        leftPercent  = (leftPercent  * ratio) / 50;
    }
    
    // Send the integer speeds (possibly negative) to the motors
    setMotorPower(MOTOR_LEFT, (int8_t) leftPercent);
    setMotorPower(MOTOR_RIGHT, (int8_t) rightPercent);
}

void setMotorPower(Motor_t motor, int8_t powerInPercent)
{
    // Determine motor direction based on sign
    if (powerInPercent < 0)
    {
        setMotorDirection(motor, false);
        // Convert to positive for further use
        powerInPercent = -powerInPercent;
    }
    else
    {
        setMotorDirection(motor, true);
    }

    // Clamp power to [0..100]
    if (powerInPercent > 100)
    {
        powerInPercent = 100;
    }

    // Now use the clamped/positive power to drive the motor
    switch (motor)
    {
        case MOTOR_LEFT:
            SET_MOTOR_LEFT(powerInPercent);
            break;
        case MOTOR_RIGHT:
            SET_MOTOR_RIGHT(powerInPercent);
            break;
    }
}

void setMotorDirection(Motor_t motor, bool forward){
    switch (motor) { 
        case MOTOR_LEFT:
            if(forward){
                MA_DIR1 = L;
                MA_DIR2 = H;
            }
            else{
                MA_DIR1 = H;
                MA_DIR2 = L;
            }
            break;
        case MOTOR_RIGHT:
            if(forward){
                MB_DIR1 = L;
                MB_DIR2 = H;
            }
            else{
                MB_DIR1 = H;
                MB_DIR2 = L;
            }
            break;
    }
}
