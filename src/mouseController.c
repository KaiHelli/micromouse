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
    moveDistance(TIMER_1, CELL_SIZE * cells, 300, 100.0f);
}

void turnLeft() {
    turnDegrees(TIMER_1, LEFT, 90, 100.0f);
}

void turnRight() {
    turnDegrees(TIMER_1, RIGHT, 90, 100.0f);
}

void turnAround() {
    turnDegrees(TIMER_1, RIGHT, 180, 100.0f);
}