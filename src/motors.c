#include <stdint.h>
#include <stdbool.h>
#include "motors.h"
#include "IOConfig.h"
#include <math.h>

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

void steerMotors(int8_t steering, float powerInPercent){
    if (powerInPercent < 0.0f) powerInPercent = 0.0f;
    if (powerInPercent > 100.0f) powerInPercent = 100.0f;
    if (steering < -100) steering = -100;
    if (steering > 100) steering = 100;
    
    float ratio = (50.0 - fabs(steering)) / 50.0;
    
     float leftPercent = powerInPercent;
     float rightPercent = powerInPercent;

    if (steering >= 0) {
        rightPercent = rightPercent  * ratio;
    } else {
        leftPercent = leftPercent  * ratio;
    }
     
    SET_MOTOR_LEFT(leftPercent);
    SET_MOTOR_RIGHT(rightPercent);
}

void turnDegrees(int16_t degrees){
    //TODO
}

void setMotorPower(Motor_t motor, float powerInPercent){
    if(powerInPercent < 0)
        setMotorDirection(motor, false);
    else
        setMotorDirection(motor, true);
     powerInPercent = fabs(powerInPercent);
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