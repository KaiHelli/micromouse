#include "mouseController.h"
#include "motorEncoders.h"
#include "fastPID.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include "globalTimers.h"
#include "move.h"

#include <stdio.h>

void moveForward(uint16_t cells) {
    //uprintf("Moving forward %d cells\r\n", cells);
    moveDistance(TIMER_1, CELL_SIZE * cells, 300, 100.0f);
}

void turnLeft() {
    //uprintf("Turning left\r\n");
    turnDegrees(TIMER_1, LEFT, 90, 100.0f);
}

void turnRight() {
    //uprintf("Turning right \r\n");
    turnDegrees(TIMER_1, RIGHT, 90, 100.0f);
}

void turnAround() {
    //uprintf("Turning around \r\n");
    //turnDegrees(TIMER_1, RIGHT, 180, 100.0f);
    turnDegrees(TIMER_1, 180, 90, 100.0f);
}
