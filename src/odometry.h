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
extern volatile float mouseVelocity[3];       // x, y, z velocity (e.g., mm/s)
extern volatile float mousePosition[3];       // x, y, z position (e.g., mm)
extern volatile float mouseAngle[3];          // pitch, roll, yaw, angles in degrees or radians

extern volatile float mouseAccelPitch;
extern volatile float mouseAccelRoll;
extern volatile float mouseMagYaw;

// Helper enums to safely access the above arrays
typedef enum {
    X = 0,
    Y,
    Z
} Axis_t;

typedef enum {
    PITCH = 0,
    ROLL,
    YAW
} Angle_t;

extern volatile uint64_t lastUpdateTimeUs;

/**
 * @brief Setup odometry to trigger every 1ms, resets data.
 */
void setupOdometry(Timer_t timer, uint16_t numTicks);

/**
 * @brief Reset all integrated states.
 */
void resetOdometry(void);

/**
 * @brief Timer callback that triggers IMU measurement reads (gyro / accel / mag / encoder).
 *
 * This function is called by a timer; it initiates asynchronous
 * reads of gyro & accel from the IMU. The completion of each read calls
 * back to odometryIMUGyroUpdate() or odometryIMUAccelUpdate().
 */
int16_t triggerOdometryUpdate();


/**
 * @brief Integrates Z-gyro measurement to maintain yaw estimate.
 *
 * Called when new gyro data are available.
 */
void odometryIMUGyroUpdate(uint8_t numSamples, float dtPerSample, float dtTotal);

/**
 * @brief Integrates accelerometer data to maintain velocity & position.
 *
 * Called when new accel data are available.
 */
void odometryIMUAccelUpdate(uint8_t numSamples, float dtPerSample, float dtTotal);

/**
 * @brief Integrates accelerometer data to maintain yaw headings.
 *
 * Called when new mag data are available.
 */
void odometryIMUMagUpdate(float dtTotal);

/**
 * @brief Integrates wheel encoder data to maintain yaw headings and position.
 *
 * Called when new wheel encoder data are available.
 */
void odometryEncoderUpdate(void);


bool robotIsStationary(void);

#endif // ODOMETRY_H
