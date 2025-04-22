#include "mouseController.h"
#include "motorEncoders.h"
#include "fastPID.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include "globalTimers.h"

#include <stdio.h>

MouseState_t mouseState = MOUSE_STATE_INITIALIZING;

MovementState_t movementState = MOVEMENT_STATE_NONE;

//static FastPid pid;


int16_t moveForward(uint16_t cells) {
    //resetPid
    //setgoaldistance numCells * celldistance
    //movement state: moving
    //registerTimerCallback(TIMERX, updatePid)
    //update pid will set movement state to NONE and return 0 (unregistering the callback) if goal reached
    //while(movementstae==moving) {} busy wait
    //
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
   
    return 1;
}


uint8_t start() {
    uint8_t status = solveMaze();
    return status; 
}