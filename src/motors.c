#include <stdint.h>
#include <stdbool.h>
#include "motors.h"
#include "IOConfig.h"

void setMotorsState(MotorState_t state){
    switch (state) { 
        case MOTORS_STANDBY:
            M_STDBY = L;
            break;
        case MOTORS_BRAKE:
            M_STDBY = H;
            MA_DIR1 = H;
            MA_DIR2 = H;
            MB_DIR1 = H;
            MB_DIR2 = H;
            break;
        case MOTORS_BACKWARD:
            M_STDBY = H;
            MA_DIR1 = H;
            MA_DIR2 = L;
            MB_DIR1 = H;
            MB_DIR2 = L;
            break;
        case MOTORS_FORWARD:
            M_STDBY = H;
            MA_DIR1 = L;
            MA_DIR2 = H;
            MB_DIR1 = L;
            MB_DIR2 = H;
            break;
        case MOTORS_ROTATE_LEFT:
            M_STDBY = H;
            MA_DIR1 = H;
            MA_DIR2 = L;
            MB_DIR1 = L;
            MB_DIR2 = H;
            break;
        case MOTORS_ROTATE_RIGHT:
            M_STDBY = H;
            MA_DIR1 = L;
            MA_DIR2 = H;
            MB_DIR1 = H;
            MB_DIR2 = L;
            break;
    }
}

void steerMotors(int8_t steering, float powerInPercent){
    if (powerInPercent < 0.0f) powerInPercent = 0.0f;
    if (powerInPercent > 100.0f) powerInPercent = 100.0f;
    if (steering < -100) steering = -100;
    if (steering > 100) steering = 100;
    
    float leftPower = (1.0f - (steering / 100.0f));
    float rightPower = (1.0f + (steering / 100.0f));
    //TODO
    SET_MOTOR_LEFT(leftPower);
    SET_MOTOR_RIGHT(rightPower);
}

void turnDegrees(int16_t degrees){
    //TODO
}

void setMotorPower(Motor_t motor, float powerInPercent){
    if (powerInPercent < 0.0f) powerInPercent = 0.0f;
    if (powerInPercent > 100.0f) powerInPercent = 100.0f;
    switch (motor) { 
        case MOTOR_LEFT:
            SET_MOTOR_LEFT(powerInPercent);
            break;
        case MOTOR_RIGHT:
            SET_MOTOR_RIGHT(powerInPercent);
            break;
    }
}