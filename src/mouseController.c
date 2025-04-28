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

void turnDirection(Direction direction) {
    //uprintf("Turning left\r\n");
    turnOrientation(TIMER_1, direction, 90, 100.0f);
}

void turnLeft() {
    turnOrientation(TIMER_1, -90, 90, 100.0f);
}

void turnRight() {
    turnOrientation(TIMER_1, 90, 90, 100.0f);
}

void turnAround() {
    turnOrientation(TIMER_1, 180, 90, 100.0f);
}
