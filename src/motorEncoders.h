#ifndef MOTORENCODERS_H
#define MOTORENCODERS_H

#include <xc.h>
#include <stdint.h>
#include <math.h>

// Definitions and Constants
#define ENC_MAX_VALUE          0x10000

#define TICKS_PER_REV          (4 * 16 * 33)  // 4x QEI, 16 pulses, 33:1 gearbox
#define WHEEL_DIAMETER_MM      60.0f
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * M_PI)
#define DIST_PER_TICK          (WHEEL_CIRCUMFERENCE_MM / TICKS_PER_REV)

#define WHEEL_BASE_MM          97.0f

#define RAD2DEG                (180.0f / M_PI)

/**
 * @brief Enumeration for selecting which encoder to use.
 */
typedef enum {
    ENCODER_LEFT,   // Refers to QEI2
    ENCODER_RIGHT   // Refers to QEI1
} MotorEncoder_t;

/**
 * @brief Initializes the specified QEI encoder.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @param startPos The initial position for the encoder.
 */
void initQEI(MotorEncoder_t encoder, uint16_t startPos);

/**
 * @brief Returns the current encoder count (including any rollover adjustments).
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Current encoder count.
 */
int32_t getPositionInCounts(MotorEncoder_t encoder);

/**
 * @brief Returns the current position in radians.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Position in radians.
 */
float getPositionInRad(MotorEncoder_t encoder);

/**
 * @brief Returns the current position in degrees.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Position in degrees.
 */
float getPositionInDeg(MotorEncoder_t encoder);

/**
 * @brief Returns the current velocity in encoder counts per sample.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Velocity in counts per sample.
 */
int16_t getVelocityInCountsPerSample(MotorEncoder_t encoder);

/**
 * @brief Returns the current rotational velocity in radians per second.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Rotational velocity in rad/s.
 */
float getVelocityInRadPerSecond(MotorEncoder_t encoder);


/**
 * @brief Returns the current rotational velocity in degrees per second.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Rotational velocity in deg/s.
 */
float getVelocityInDegPerSecond(MotorEncoder_t encoder);

/**
 * @brief Computes the approximate yaw (in degrees) based on the difference
 *        between the right and left encoder distances.
 * @return Yaw in degrees.
 */
float getEncoderYawDeg(void);

#endif /* MOTORENCODERS_H */
