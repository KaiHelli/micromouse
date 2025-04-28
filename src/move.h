#ifndef MOVE_H
#define	MOVE_H

#include <stdint.h>

#include "timers.h"

typedef enum {
    UP = 0,
    RIGHT,
    DOWN,
    LEFT
} Direction;

/**
 * @brief Rotates the robot by the specified angle in degrees on the spot.
 * Takes an int16_t degrees value; positive rotates CW, negative CCW.
 * The powerInPercent parameter (float) defines the overall base power level
 * as a percentage of the motor's maximum (0 for no power, 100 for full power).
 * Registers a timer callback on the stated timer to check for turn completion
 * periodically.
 */
void turnDegrees(Timer_t timer, int16_t degrees, float cruiseDegPerSec, float timer_hz);

void turnOrientation(Timer_t timer, Direction direction, float cruiseDegPerSec, float timer_hz);

/**
 * @brief TODO
 */
void moveDistance(Timer_t timer, int16_t distance, float cruise_mmps, float timer_hz);

void calibrateGlobalOrientation();

#endif	/* MOVE_H */

