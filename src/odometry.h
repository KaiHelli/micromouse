#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include "timers.h"

/**
 * @brief Global odometry state (velocity, position, yaw).
 *
 * These are declared as extern here for visibility. Make sure
 * the corresponding definitions in the .c file are declared
 * without `static` so that there is only one instance of each.
 */
extern volatile float velocity[3];       // x, y, z velocity (e.g., mm/s)
extern volatile float position[3];       // x, y, z position (e.g., mm)
extern volatile float yaw;               // Yaw angle in degrees or radians

extern volatile uint64_t lastUpdateTimeUs;

/**
 * @brief Setup odometry to trigger every 1ms, resets data.
 */
void setupOdometry(Timer_t timer);

/**
 * @brief Reset all integrated states.
 */
void resetOdometry(void);

/**
 * @brief Timer callback that triggers IMU measurement reads every 1 ms.
 *
 * Initiates asynchronous reads of gyro & accel from the IMU.
 */
int16_t triggerIMUMeasurements(void);

/**
 * @brief Integrates Z-gyro measurement to maintain yaw estimate.
 *
 * Called when new gyro data are available.
 */
void odometryIMUGyroUpdate(void);

/**
 * @brief Integrates accelerometer data to maintain velocity & position.
 *
 * Called when new accel data are available.
 */
void odometryIMUAccelUpdate(void);

/**
 * @brief Optional motor-encoder odometry fusion.
 */
void odometryMotorEncoderUpdate(void);

bool robotIsStationary(void);

#endif // ODOMETRY_H
