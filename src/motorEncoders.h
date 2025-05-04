#ifndef MOTORENCODERS_H
#define MOTORENCODERS_H

#include <xc.h>
#include <stdint.h>
#include <math.h>

// Definitions and Constants
#define ENC_MAX_VALUE          0x10000

/**
 * @brief Enumeration for selecting which encoder to use.
 * 
 * Note: According to the implementation, ENCODER_LEFT is associated with QEI1 and
 *       ENCODER_RIGHT is associated with QEI2.
 */
typedef enum {
    ENCODER_LEFT = 0,   // Uses QEI1
    ENCODER_RIGHT       // Uses QEI2
} MotorEncoder_t;


void initMotorEncoders(void);

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
int32_t getEncoderPositionCounts(MotorEncoder_t encoder);

/**
 * @brief Returns the current position in radians.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Position in radians.
 */
float getEncoderPositionRad(MotorEncoder_t encoder);

/**
 * @brief Returns the current position in degrees.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Position in degrees.
 */
float getEncoderPositionDeg(MotorEncoder_t encoder);

/**
 * @brief Updates the encoder velocity measurements.
 * This function should be called periodically (e.g., from a timer interrupt)
 * to update velocity values based on the new encoder positions.
 */
int16_t updateEncoderVelocities(void);

int16_t updateEncoderVelocitiesNaive(float hz);

/**
 * @brief Returns the current rotational velocity in counts per sample.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Rotational velocity in counts since the last sample.
 */
int32_t getEncoderVelocityCountsPerSample(MotorEncoder_t encoder);

/**
 * @brief Returns the current rotational velocity in radians per second.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Rotational velocity in rad/s.
 */
float getEncoderVelocityRadPerSec(MotorEncoder_t encoder);

/**
 * @brief Returns the current rotational velocity in degrees per second.
 * @param encoder ENCODER_LEFT or ENCODER_RIGHT.
 * @return Rotational velocity in deg/s.
 */
float getEncoderVelocityDegPerSec(MotorEncoder_t encoder);

float getEncoderVelocityMmPerSec(MotorEncoder_t encoder);

/**
 * @brief Computes the current yaw rate in radians per second based on the difference
 *        between the left and right encoder velocities.
 * @return Yaw rate in radians per second.
 */
float getEncoderYawRateRadPerSec(void);

float getEncoderLinearVelocityMmPerSec(void);

float getEncoderAverageDistanceMm(void);

float getEncoderAverageDistanceUm(void);

void getEncoderLinearVelocityAndYawRate(float* linearVelocityMmPerSec, float* yawRateRadPerSec);


#endif /* MOTORENCODERS_H */
