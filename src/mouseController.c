#include "mouseController.h"
#include "motorEncoders.h"
#include "fastPID.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include <stdio.h>

MouseState_t mouseState = MOUSE_STATE_INITIALIZING;

MovementState_t movementState = MOVEMENT_STATE_NONE;

static FastPid pid;


int16_t moveForward() {
    //resetPid
    //setgoaldistance numCells * celldistance
    //movement state: moving
    //registerTimerCallback(TIMERX, updatePid)
    //update pid will set movement state to NONE and return 0 (unregistering the callback) if goal reached
    //while(movementstae==moving) {} busy wait
    //
    
    uint16_t left = getSensorDistance(SENSOR_LEFT);
    uint16_t right = getSensorDistance(SENSOR_RIGHT);
    uint16_t front = getSensorDistance(SENSOR_CENTER);
    
    int16_t step = fastPidStep(&pid, 0, (int16_t) left-right);
    
    uint8_t defaultPower = 60;
    
    int8_t powerLeft = defaultPower + (int8_t) step;
    int8_t powerRight = defaultPower - (int8_t) step;
    
    setMotorPower(MOTOR_RIGHT, powerRight);
    setMotorPower(MOTOR_LEFT, powerLeft);
    
    //char buffer[100];
    //snprintf(buffer, sizeof(buffer), "Power [%] - Step: %d, Left: %d, Right: %d\r\n", (int8_t) step, powerLeft, powerRight);
    //putsUART1(buffer);
    //snprintf(buffer, sizeof(buffer), "Sensor readings in mm left: %u, right: %u, center: %u\r\n",left, right, front);
    //putsUART1(buffer);
    return 1;
}

uint8_t turnLeft() {
    //resetPid
    //setgoalangle 90
    //movement state: turning
    //registerTimerCallback(TIMERX, updatePid)
    //update pid will set movement state to NONE and return 0 (unregistering the callback) if goal reached
    //while(movementstae==turning) {} busy wait
    //
    return 1;
}

uint8_t turnRight() {
    //resetPid
    //setgoalangle -90
    //movement state: turning
    //registerTimerCallback(TIMERX, updatePid)
    //update pid will set movement state to NONE and return 0 (unregistering the callback) if goal reached
    //while(movementstae==turning) {} busy wait
    //
    return 1;
}

uint8_t initMouseState() {
    //set state to standby, LED1:ON 
    //button callback: discovery
    bool status = true;
    
    fastPidInit(&pid);
    //status &= fastPidConfigure(&pid, 0.6f, 0.006f, 0.0f, 3.3333f, 8, true);
    status &= fastPidConfigure(&pid, 0.8f, 0.0f, 0.0f, 100.0f, 8, true);
    status &= fastPidSetOutputRange(&pid, -100, 100);
            
    if (!status) {
        putsUART1("PID setup failed.\r\n");
    }
    
    return 1;
}

uint8_t startDiscovery() {
    //check if in standby state else do not start discovery
    //state:discovery, LED2:ON
    //start flood fill, on success or error will set mouse state accordingly
    //while(state==discovery) {} busy wait
    //if target found
        //change button callback to run
        //LED3: ON
    //else 
        //state:standby
        //LED1,2 OFF
    return 1;
}

uint8_t startRun() {
    //check if state==tagetFound
    //set state to run, LED4:ON
    //start command sequence execution found during flood fill
    //while(state==run) {} busy wait
    //if target reached: LED5 ON
    //else: LED1-5 off
    return 1;
}

