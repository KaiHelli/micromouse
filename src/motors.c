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

volatile uint8_t powerPctLeft;
volatile uint8_t powerPctRight;

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
            powerPctLeft = powerInPercent;
            break;
        case MOTOR_RIGHT:
            SET_MOTOR_RIGHT(powerInPercent);
            powerPctRight = powerInPercent;
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
