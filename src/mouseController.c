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
    turnOrientation(TIMER_1, direction, 90, 100.0f);
}

void turnLeft() {
    turnDegrees(TIMER_1, -90, 90, 100.0f);
}

void turnRight() {
    turnDegrees(TIMER_1, 90, 90, 100.0f);
}

void turnAround() {
    turnDegrees(TIMER_1, 180, 90, 100.0f);
}
