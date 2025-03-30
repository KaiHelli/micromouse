#ifndef MOTORENCODERS_H
#define MOTORENCODERS_H

#include <xc.h>
#include <stdint.h>

/**
 * @brief Initializes QEI1 using the specified start position.
 * The 'startPos' parameter sets the initial position for QEI1.
 */
void initQEI1(uint16_t startPos);

/**
 * @brief Initializes QEI2 using the specified start position.
 * The 'startPos' parameter sets the initial position for QEI2.
 */
void initQEI2(uint16_t startPos);

/**
 * @brief Returns the current position in radians based on encoder data.
 */
float getPositionInRad();

/**
 * @brief Returns the current rotational velocity in radians per second.
 */
float getVelocityInRadPerSecond();

/**
 * @brief Returns the current position (QEI1) in encoder counts.
 */
int32_t getPositionInCounts_1();

/**
 * @brief Returns the current velocity (QEI1) in encoder counts per sample.
 */
int16_t getVelocityInCountsPerSample_1();

/**
 * @brief Returns the current position (QEI2) in encoder counts.
 */
int32_t getPositionInCounts_2();

/**
 * @brief Returns the current velocity (QEI2) in encoder counts per sample.
 */
int16_t getVelocityInCountsPerSample_2();

extern int32_t rotationCount1;
extern int32_t rotationCount2;

#define GET_ENCODER_1(RIGHT_ENCODER_POSITION_VALUE) (RIGHT_ENCODER_POSITION_VALUE = rotationCount1 + POSCNT)
#define GET_ENCODER_2(LEFT_ENCODER_POSITION_VALUE) (LEFT_ENCODER_POSITION_VALUE = rotationCount2 + POS2CNT)

#define WHEEL_ROTATIONS_PERROBOT_ROTATION 2.5
#define TICKS_PER_WHEELROTATION (64 * 33)
#define TICKS_PER_CENTIMETER TICKS_PER_WHEELROTATION / 12.566
#define METER_PER_TICkS 0.12566 / TICKS_PER_WHEELROTATION
#define DELTATICKS_90_DEGREES (0.25 * WHEEL_ROTATIONS_PERROBOT_ROTATION * TICKS_PER_WHEELROTATION)
#define DELTATICKS_180_DEGREES (0.5 * WHEEL_ROTATIONS_PERROBOT_ROTATION * TICKS_PER_WHEELROTATION)
#define DELTATICKS_CELL_GAP (11.5 * TICKS_PER_CENTIMETER)

#endif /* MOTORENCODERS_H */