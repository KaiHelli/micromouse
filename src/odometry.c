#include "odometry.h"
#include "imu.h"
#include "globalTimers.h"
#include "timers.h"

#include <math.h>    // For sinf, cosf if you choose to use them.

#define G_TO_MM_S2             9.80665f * 1000.0f

volatile float velocity[3]       = {0.0f, 0.0f, 0.0f};
volatile float position[3]       = {0.0f, 0.0f, 0.0f};
volatile float yaw               = 0.0f; // Yaw around Z, in radians or degrees.

volatile uint64_t lastUpdateTimeUs = 0;

/**
 * @brief Setup odometry to trigger every 1ms, resets data.
 */
void setupOdometry(Timer_t timer)
{
    resetOdometry();
    // Register IMU measurements on specified timer.
    registerTimerCallback(timer, triggerIMUMeasurements);
}

/**
 * @brief Reset all integrated states.
 */
void resetOdometry(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        velocity[i]  = 0.0f;
        position[i]  = 0.0f;
    }
    yaw               = 0.0f;
    lastUpdateTimeUs  = getTimeInUs(); // Reset reference time
}

/**
 * @brief Timer callback that triggers IMU measurement reads every 1 ms.
 *
 * This function is called by the 1?kHz timer; it initiates asynchronous
 * reads of gyro & accel from the IMU. The completion of each read calls
 * back to odometryIMUGyroUpdate() or odometryIMUAccelUpdate().
 */
int16_t triggerIMUMeasurements()
{
    // Will trigger async polls of gyro / accel readings from the IMU.
    // Each call will call back to odometryIMUGyroUpdate / odometryIMUAccelUpdate
    // once new measurements are available to process.

    imuReadGyro();
    imuReadAccel();
    
    return 1;
}

// -------------------------------------------------------------------------
// GYRO UPDATE
// -------------------------------------------------------------------------
void odometryIMUGyroUpdate(void)
{
    // Called when new gyro data are available in:
    //   extern volatile int16_t rawGyroMeasurements[3]; -> raw readings from sensor
    // We want to integrate the Z-axis gyro to maintain a yaw estimate.

    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs       = currentTimeUs - lastUpdateTimeUs;
    lastUpdateTimeUs       = currentTimeUs;

    // Convert microseconds to seconds in fixed or float format:
    // Minimizing float usage, but one float multiplication for dt is acceptable:
    float dt = (float) deltaUs * 1.0e-6f;  // seconds

    // Scale the raw Z gyro reading to deg/s.
    float yaw_dps;
    imuScaleGyroMeasurement(&rawGyroMeasurements[2], &yaw_dps);

    // Integrate yaw:
    yaw += yaw_dps * dt;  // [degrees]

    // Keep yaw in [0, 360) to avoid numeric blowup
    if (yaw > 360.0f) {
        yaw -= 360.0f;
    } else if (yaw < 0) {
        yaw += 360.0f;
    }
}

// -------------------------------------------------------------------------
// ACCEL UPDATE
// -------------------------------------------------------------------------
void odometryIMUAccelUpdate(void)
{
    // Called when new accel data are available in:
    //   extern volatile int16_t rawAccelMeasurements[3]; -> raw readings from sensor
    // We'll integrate acceleration to update velocity & position in the x-y plane.

    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs       = currentTimeUs - lastUpdateTimeUs;
    lastUpdateTimeUs       = currentTimeUs;

    float dt = (float)deltaUs * 1.0e-6f;  // seconds

    // Scale raw accelerometer to g?s:
    
    float accelX_g;
    float accelY_g;
    
    imuScaleAccelMeasurement(&rawAccelMeasurements[0], &accelX_g);
    imuScaleAccelMeasurement(&rawAccelMeasurements[1], &accelY_g);

    // Convert x,y from g to m/s^2. If your robot runs natively in mm/s^2,
    // scale accordingly. For example in m/s^2:
    float accelX_mm_s2 = accelX_g * G_TO_MM_S2;
    float accelY_mm_s2 = accelY_g * G_TO_MM_S2;

    // Note: No local to global frame conversion done for now.
    
    // Integrate to get velocity in global frame (x,y):
    velocity[0] += accelX_mm_s2 * dt;  // vx
    velocity[1] += accelY_mm_s2 * dt;  // vy

    // Integrate velocity to get position in global frame (x,y):
    position[0] += velocity[0] * dt;    // x
    position[1] += velocity[1] * dt;    // y
}

// -------------------------------------------------------------------------
// OPTIONAL: Motor Encoder Updates (Fusion)
// -------------------------------------------------------------------------
void odometryMotorEncoderUpdate(void)
{
    // If you have quadrature encoders on drive motors, you can read them here
    // (possibly at a lower rate), convert ticks to linear distance, and fuse
    // the result with the IMU-based estimates.
}