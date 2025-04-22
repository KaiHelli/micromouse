#ifndef MOUSECONTROLLER_H
#define	MOUSECONTROLLER_H

#include <xc.h>

typedef enum {
    MOUSE_STATE_INITIALIZING,
    MOUSE_STATE_STANDBY, //LED1 ON, push button: Start discovery
    MOUSE_STATE_RUNNING, //LED1,2,3,4 ON
    MOUSE_STATE_TARGET_REACHED, //LED1-5 ON
    MOUSE_STATE_ERROR //Discovery or run failed, LED1-5 OFF, push button: Start discovery
} MouseState_t;

typedef enum {
    MOVEMENT_STATE_NONE, 
    MOVEMENT_STATE_MOVING,
    MOVEMENT_STATE_TURNING
} MovementState_t;

//Movement API
int16_t moveForward(uint16_t cells);

uint8_t turnLeft();

uint8_t turnRight();

//Mouse high level API

uint8_t start();

uint8_t initMouseState();

#endif	/* MOUSECONTROLLER_H */

