#include "mouseController.h"
#include "motorEncoders.h"
#include "fastPID.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include "globalTimers.h"
#include "move.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>


#define SENSOR_FRONT_THRESHOLD 29
#define SENSOR_OFFSET_THRESHOLD 1

void centerMouseInCell() {
    uprintf("Center mouse in cell.\r\n");
    int32_t lrOffset = 0;
    int32_t frOffset = 0;
    do {
        uint16_t left  = getSensorDistance(SENSOR_LEFT);
        uint16_t right = getSensorDistance(SENSOR_RIGHT);
        uint16_t front = getSensorDistance(SENSOR_CENTER);

        lrOffset = (int32_t) right - (int32_t) left;
        frOffset = (int32_t) front - (int32_t) SENSOR_FRONT_THRESHOLD;
        uprintf("LR Offset: %ld | FR Offset: %ld\r\n", lrOffset, frOffset);
        uprintf("L: %u F: %u R: %u\r\n", left, front, right);
        
        __delay_ms(500);
    } while (labs(lrOffset) > SENSOR_OFFSET_THRESHOLD || labs(frOffset) > SENSOR_OFFSET_THRESHOLD);
    
    uprintf("Centering done. Don't move anymore.\r\n");
}


void moveForward(uint16_t cells) {
    //uprintf("Moving forward %d cells\r\n", cells);
    moveDistance(TIMER_1, CELL_SIZE * cells, 300, 2, getTimerFrequency(TIMER_1));
}

void turnDirection(Direction direction) {
    turnOrientation(TIMER_1, direction, 90, 2, getTimerFrequency(TIMER_1));
}

void turnLeft() {
    turnDegrees(TIMER_1, -90, 90, 2, getTimerFrequency(TIMER_1));
}

void turnRight() {
    turnDegrees(TIMER_1, 90, 90, 2, getTimerFrequency(TIMER_1));
}

void turnAround() {
    turnDegrees(TIMER_1, 180, 90, 2, getTimerFrequency(TIMER_1));
}
