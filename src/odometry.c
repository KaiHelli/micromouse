#include "odometry.h"
#include "imu.h"
#include "globalTimers.h"
#include "timers.h"
#include "motorEncoders.h"
#include "IOconfig.h"
#include "constants.h"
#include "uart.h"

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define G_TO_MM_S2             9.80665f * 1000.0f

volatile float mouseVelocity[3]       = {0.0f, 0.0f, 0.0f};
volatile float mousePosition[3]       = {0.0f, 0.0f, 0.0f};
volatile float mouseAngle[3]          = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw

volatile float localVelocity[3]       = {0.0f, 0.0f, 0.0f};
volatile float localPosition[3]       = {0.0f, 0.0f, 0.0f};
volatile float localAngle[3]          = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw

volatile uint64_t lastGyroUpdateTimeUs  = 0;
volatile uint64_t lastAccelUpdateTimeUs = 0;
volatile uint64_t lastEncoderUpdateTimeUs = 0;

bool initAccelEstimates = true;
bool initMagEstimates = true;

/**
 * @brief Setup odometry to trigger every 1ms, resets data.
 */
void setupOdometry(Timer_t fastTimer, Timer_t slowTimer)
{
    resetOdometry();
    // Register fast IMU updates for gyroscope and accelerometer (200Hz)
    registerTimerCallback(fastTimer, triggerGyroAccelUpdate);
    
    // Register slower IMU updates for magnetometer and slower quadrature
    // encoder updates (100 Hz) .
    registerTimerCallback(slowTimer, triggerEncoderMagUpdate);
}

/**
 * @brief Reset all integrated states.
 */
void resetOdometry(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        mouseVelocity[i]  = 0.0f;
        mousePosition[i]  = 0.0f;
        mouseAngle[i]     = 0.0f;
        
        localVelocity[i]  = 0.0f;
        localPosition[i]  = 0.0f;
        localAngle[i]     = 0.0f;
    }
    lastGyroUpdateTimeUs  = getTimeInUs(); // Reset reference time
    lastAccelUpdateTimeUs = lastGyroUpdateTimeUs;
}

/**
 * @brief Timer callback that triggers IMU measurement reads (gyro / accel).
 *
 * This function is called by a timer; it initiates asynchronous
 * reads of gyro & accel from the IMU. The completion of each read calls
 * back to odometryIMUGyroUpdate() or odometryIMUAccelUpdate().
 */
int16_t triggerGyroAccelUpdate()
{
    // Will trigger async polls of gyro / accel readings from the IMU.
    // Each call will call back to odometryIMUGyroUpdate / odometryIMUAccelUpdate
    // once new measurements are available to process.
    imuReadGyro();
    imuReadAccel();
    
    return 1;
}

/**
 * @brief Timer callback that triggers IMU measurement reads (mag) and updates
 * quadrature encoder estimates.
 *
 * This function is called by a timer; it initiates asynchronous
 * reads of mage from the IMU. The completion of each read calls
 * back to odometryIMUMagUpdate().
 */
int16_t triggerEncoderMagUpdate()
{
    // Will trigger async polls of magnetometer readings from the IMU.
    // Each call will call back to odometryIMUMagUpdate once new measurements
    // are available to process.
    imuReadMag();
    updateEncoderVelocities();
    odometryEncoderUpdate();
    
    return 1;
}

static inline float wrapPi(float angle)
{
    if      (angle >=  M_PI) angle -= 2.0f*M_PI;
    else if (angle <  -M_PI) angle += 2.0f*M_PI;
    
    return angle;
}

/**
 * Fuse two angles (prev and measured) with a complementary filter,
 * wrapping the error over the ±? boundary and keeping the result in [0?2?).
 */
static float fuseAngle(float prev, float measured, float alpha) {
    // 1) short?way signed error in (???+?]
    float err = measured - prev;
    if      (err >  M_PI) err -= 2.0f * M_PI;
    else if (err < -M_PI) err += 2.0f * M_PI;

    // 2) complementary update
    prev += (1.0f - alpha) * err;

    prev = wrapPi(prev);

    return prev;
}

// -------------------------------------------------------------------------
// GYRO UPDATE ~250us
// -------------------------------------------------------------------------
void odometryIMUGyroUpdate(void)
{
    // Called when new gyro data are available in:
    //   extern volatile int16_t rawGyroMeasurements[3]; -> raw readings from sensor
    // We want to integrate the Z-axis gyro to maintain a yaw estimate.

    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs       = currentTimeUs - lastGyroUpdateTimeUs;
    lastGyroUpdateTimeUs   = currentTimeUs;

    // Zero-Rotation Update (ZUPT)
    // if(robotIsStationary()) {
        // Early return: no need to integrate rotation when stationary
    //    return;
    //}
    
    // Convert microseconds to seconds in fixed or float format:
    float dt = (float) deltaUs * 1.0e-6f;  // seconds

    // Scale the raw Z gyro reading to deg/s.
    float angles_dps[3];
    imuScaleGyroMeasurements(rawGyroMeasurements, angles_dps);

    // Integrate angles:
    for (uint8_t axis = 0; axis < 3; axis++) {
        localAngle[axis] = mouseAngle[axis] + angles_dps[axis] * dt * DEG2RAD; // TODO!
        
        // Wrap into [????)
        localAngle[axis] = wrapPi(localAngle[axis]);
    }
}

// -------------------------------------------------------------------------
// ACCEL UPDATE ~650us
// -------------------------------------------------------------------------
void odometryIMUAccelUpdate(void)
{
    // Called when new accel data are available in:
    //   extern volatile int16_t rawAccelMeasurements[3]; -> raw readings from sensor
    // We'll integrate acceleration to update velocity & position in the x-y plane.

    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs       = currentTimeUs - lastAccelUpdateTimeUs;
    lastAccelUpdateTimeUs  = currentTimeUs;

    float dt = (float)deltaUs * 1.0e-6f;  // seconds

    // Scale raw accelerometer to g's:
    float accel_g[3];
    
    imuCalibrateAccelMeasurements(rawAccelMeasurements, accel_g);
    imuScaleAccelMeasurementsFloat(accel_g, accel_g);
    
    /*
    static uint32_t i = 0;
    if (i % 20 == 0) {
        char measurementStr[70];
        snprintf(measurementStr, 70, "Accelerometer [g]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);
        putsUART1(measurementStr);
    }
    i++;
    */
    
    // Calculate roll and pitch estimates
    float roll = -atan2f(accel_g[X], accel_g[Z]); // atan(x/z)
    float pitch = atan2f(-accel_g[Y], sqrtf(accel_g[X]*accel_g[X] + accel_g[Z]*accel_g[Z])); // atan(y/sqrt(x^2+z^2))
    
    // Initialize estimates
    if (initAccelEstimates) {
        mouseAngle[PITCH] = pitch;
        mouseAngle[ROLL] = roll;
        
        initAccelEstimates = false;
    }
    
    // Fuse with gyroscope readings (complementary filter)
    const float alpha = 0.98f;
    mouseAngle[PITCH] = fuseAngle(localAngle[PITCH], pitch, alpha);
    mouseAngle[ROLL]  = fuseAngle(localAngle[ROLL],  roll,  alpha);
    
    // Yaw rate will be improved by magnetometer and wheel encoders at a slower rate
    mouseAngle[YAW] = localAngle[YAW];
    
    // Zero-Velocity Update (ZUPT)
    /*
    if(robotIsStationary()) {
        mouseVelocity[0] = 0.0f;
        mouseVelocity[1] = 0.0f;
        mouseVelocity[2] = 0.0f;
        
        // Early return: no need to integrate velocity and position when stationary
        return;
    }
    
    // GRAVITY COMPENSATION  (all math done in g?s)
    //   body?frame gravity from current roll & pitch:
    //        g_b = [?sin?,  sin?·cos?,  cos?·cos?]
    float rollSin = sinf(mouseAngle[ROLL]);
    float rollCos = cosf(mouseAngle[ROLL]);
    float pitchSin = sinf(mouseAngle[PITCH]);
    float pitchCos = cosf(mouseAngle[PITCH]);

    float g_bx = -pitchSin;
    float g_by =  rollSin * pitchCos;
    //float g_bz =  rollCos * pitchCos;

    // specific force (still in g?s)
    float f_gx = accel_g[X] - g_bx;
    float f_gy = accel_g[Y] - g_by;
    
    char buf[40];
    snprintf(buf, sizeof(buf), "f_gx=%f  f_gy=%f\r\n", f_gx, f_gy);
    putsUART1(buf);

    // Z is not used for 2?D odometry, but you could keep it:
    // float f_gz = a_g[Z] - g_bz;

    // Convert x,y from g to m/s^2.
    float accelX_mm_s2 = f_gx * G_TO_MM_S2;
    float accelY_mm_s2 = f_gy * G_TO_MM_S2;
    
    // Transform accelerations from local robot frame to global frame using yaw.
    float cosYaw = cosf(mouseAngle[YAW]);
    float sinYaw = sinf(mouseAngle[YAW]);
    
    // local (robot) axes: y forward, x left/right
    float accelGlobalX = accelX_mm_s2 * cosYaw - accelY_mm_s2 * sinYaw;
    float accelGlobalY = accelX_mm_s2 * sinYaw + accelY_mm_s2 * cosYaw;
    
    // TODO: For now we only track in a local frame.
    // float accelGlobalX = accelX_mm_s2;
    // float accelGlobalY = accelY_mm_s2;
    
    // Integrate to get velocity in global frame (x,y):
    mouseVelocity[0] += accelGlobalX * dt;  // vx
    mouseVelocity[1] += accelGlobalY * dt;  // vy

    // Integrate velocity to get position in global frame (x,y):
    mousePosition[0] += mouseVelocity[0] * dt;    // x
    mousePosition[1] += mouseVelocity[1] * dt;    // y
    */
}


// -------------------------------------------------------------------------
// MAG UPDATE ~440us
// -------------------------------------------------------------------------
void odometryIMUMagUpdate(void) {
    float scaledMagMeasurements[3];
    
    // Get calibrated magnetometer readings
    imuCalibrateMagMeasurements(rawMagMeasurements, scaledMagMeasurements);
    
    // Get magnetometer heading using pitch and roll estimates
    //float yaw = magnetometerToHeading(scaledMagMeasurements);
    float magYaw = magnetometerToTiltCompensatedHeading(scaledMagMeasurements, mouseAngle[PITCH], mouseAngle[ROLL]);

    // Initialize estimates
    if (initMagEstimates)
    {
        mouseAngle[YAW] = magYaw;
        initMagEstimates = false;
    }
    
    // Fuse heading with estimated yaw of gyroscope
    const float alpha = 0.98f;
    localAngle[YAW] = fuseAngle(localAngle[YAW], magYaw, alpha);
    mouseAngle[YAW] = localAngle[YAW];
}

// -------------------------------------------------------------------------
// ENCODER UPDATE ~220us
// -------------------------------------------------------------------------
void odometryEncoderUpdate(void)
{
    static float yawAtLastEncUpdate = 0.0f;
    static bool  encInit = false;
    
    uint64_t currentTimeUs = getTimeInUs();
    uint64_t deltaUs = currentTimeUs - lastEncoderUpdateTimeUs;
    lastEncoderUpdateTimeUs = currentTimeUs;

    if (!encInit) {
        yawAtLastEncUpdate = mouseAngle[YAW];
        encInit = true;
        return;
    }
    
    float dt = (float)deltaUs * 1.0e-6f;
    float yawRateBody;
    float linearVelocityBody; 
    
    getEncoderLinearVelocityAndYawRate(&linearVelocityBody, &yawRateBody);
    
    /* ---- compensate for tilt ---------------------------------------- */
    // float cosP = cosf(mouseAngle[PITCH]);
    // float cosR = cosf(mouseAngle[ROLL]);
    // float yawRateWorld = yawRateBody * cosP * cosR;     // around world z
    /* ------------------------------------------------------------------ */

    /* integrate only once, starting from the cached value */
    float encoderYaw = yawAtLastEncUpdate + yawRateBody * dt;
    
    encoderYaw = wrapPi(encoderYaw);
    
    const float alphaEnc = 0.9f;                      // trust IMU more
    localAngle[YAW] = fuseAngle(localAngle[YAW], encoderYaw, alphaEnc);
    mouseAngle[YAW] = localAngle[YAW];                // keep gyro seed aligned
    
    /* save for the *next* cycle */
    yawAtLastEncUpdate = mouseAngle[YAW];
    
    
    //mouseVelocity[Y] = linearVelocityBody;
    //mousePosition[Y] += mouseVelocity[Y] * dt;
    
    /* ---------- NEW: transform encoder velocity to world frame ---------- */
    float yaw = mouseAngle[YAW];      // current CW yaw
    float cosYaw = cosf(yaw);
    float sinYaw = sinf(yaw);

    /* body?frame vel: +Y = forward, +X = right  (v_bx = 0 here)   */
    float vx_global =  linearVelocityBody * sinYaw;   // +X world
    float vy_global =  linearVelocityBody * cosYaw;   // +Y world

    mouseVelocity[X] = vx_global;
    mouseVelocity[Y] = vy_global;

    /* integrate to get global position */
    mousePosition[X] += vx_global * dt;
    mousePosition[Y] += vy_global * dt;
    
    //uint64_t elapsedTimeInUs = getTimeInUs() - currentTimeUs;
    //char elapsedStr[30];
    //snprintf(elapsedStr, sizeof(elapsedStr), "%llu\r\n" , elapsedTimeInUs);
    //putsUART1(elapsedStr);
}


bool robotIsStationary(void) {
    
    static uint64_t lastCheckTimeUs = 0;
    
    static int32_t lastEncoder1 = 0;
    static int32_t lastEncoder2 = 0;
    
    static bool lastResult = true;

    uint64_t currentTimeUs = getTimeInUs();

    const uint64_t intervalUs = 50000; // 50 ms
    
    if (currentTimeUs - lastCheckTimeUs < intervalUs) {
        // Return previous stationary state between intervals
        return lastResult;
    }

    int32_t currentEncoder1 = getEncoderPositionCounts(ENCODER_LEFT);
    int32_t currentEncoder2 = getEncoderPositionCounts(ENCODER_RIGHT);

    int32_t diff1 = abs(currentEncoder1 - lastEncoder1);
    int32_t diff2 = abs(currentEncoder2 - lastEncoder2);

    const int32_t stationaryThreshold = 2;

    lastResult = (diff1 <= stationaryThreshold) && (diff2 <= stationaryThreshold);

    lastEncoder1 = currentEncoder1;
    lastEncoder2 = currentEncoder2;
    lastCheckTimeUs = currentTimeUs;

    return lastResult;
}