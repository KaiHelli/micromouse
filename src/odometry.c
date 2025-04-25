#include "odometry.h"
#include "imu.h"
#include "globalTimers.h"
#include "timers.h"
#include "motorEncoders.h"
#include "IOconfig.h"
#include "constants.h"
#include "uart.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define MAG_INIT_SAMPLES 150

#define CF_TAU_PITCHROLL   0.25f      // 250 ms - accel vs. gyro
#define CF_TAU_YAW_MAG     15.0f      // 15 s - mag vs. gyro
#define CF_TAU_YAW_ENC     1.0f       // 200 ms - encoder vs. gyro
#define FAST_TURN_RAD_S    (60.0f * DEG2RAD)   // ~60 °/s  (skip mag during fast turns)

#define G_TO_MM_S2             9.80665f * 1000.0f

volatile float mouseVelocity[3]       = {0.0f, 0.0f, 0.0f};
volatile float mousePosition[3]       = {0.0f, 0.0f, 0.0f};
volatile float mouseAngle[3]          = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw

volatile float localVelocity[3]       = {0.0f, 0.0f, 0.0f};
volatile float localPosition[3]       = {0.0f, 0.0f, 0.0f};
volatile float localAngle[3]          = {0.0f, 0.0f, 0.0f}; // pitch, roll, yaw

volatile float mouseAccelPitch = 0.0f;
volatile float mouseAccelRoll = 0.0f;
volatile float mouseMagYaw = 0.0f;

volatile uint64_t lastGyroUpdateTimeUs  = 0;
volatile uint64_t lastAccelUpdateTimeUs = 0;
volatile uint64_t lastMagUpdateTimeUs = 0;       // mag ?t
static   float    lastYawRate         = 0.0f;    // rad / s from gyro Z
volatile uint64_t lastEncoderUpdateTimeUs = 0;

volatile bool encInit = false;

volatile bool initAccelEstimates = true;
volatile bool initMagEstimates = true;

volatile bool initDone = false;

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
    
    // Initialize PITCH / ROLL estimates
    while (!initDone) { 
        /* spin until PITCH / ROLL / YAW estimates are initialized */ 
    }
    
    putsUART1Str("Odometry initialized.\r\n");
    
    // TODO: Encoder PID runs at wrong rate
    // TODO: Might execute encoder update only every 2nd timer call
    // TODO: Unify timers?
    // TODO: Might increase dlpf of acc since only used for pitch / roll
    // TODO: Might turn acc calibration on again
}

/**
 * @brief Reset all integrated states.
 */
void resetOdometry(void) {
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
    lastMagUpdateTimeUs = lastGyroUpdateTimeUs;
    lastEncoderUpdateTimeUs = lastGyroUpdateTimeUs;
    
    encInit = false;
    initAccelEstimates = initMagEstimates = true;
    initDone = false;
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
    //imuReadGyro();
    //imuReadAccel();
    //imuReadTemp();
    imuReadFifo();
    
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
    //imuReadMag();
    
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
 * wrapping the error over the ±pi boundary and keeping the result in [-pi, pi).
 */
static float fuseAngle(float prev, float measured, float dt, float tau)
{
    float err = measured - prev;
    if      (err >  M_PI) err -= 2.0f*M_PI;
    else if (err < -M_PI) err += 2.0f*M_PI;

    const float alpha = tau / (tau + dt);        // dt-adaptive coefficient
    prev += (1.0f - alpha) * err;
    
    return wrapPi(prev);
}

// -------------------------------------------------------------------------
// GYRO UPDATE ~250us
// -------------------------------------------------------------------------
void odometryIMUGyroUpdate(uint8_t numSamples, float dtPerSample, float dtTotal)
{ 
    // Called when new gyro data are available in:
    //   extern volatile int16_t rawFifoGyroMeasurements[3]; -> averaged raw readings from sensor since last FIFO pull
    
    if (!initDone) {
        return;
    }
    
    // Zero-Rotation Update (ZUPT)
    // if(robotIsStationary()) {
        // Early return: no need to integrate rotation when stationary
    //    return;
    //}
    
    // Scale the raw Z gyro readings to deg/s.
    float angles_dps[3];
    imuScaleGyroMeasurementsFloat(rawFifoGyroMeasurements, angles_dps);
    
    lastYawRate = (angles_dps[YAW] * DEG2RAD);
    
    // Integrate angles:
    for (uint8_t axis = 0; axis < 3; axis++) {
        localAngle[axis] = mouseAngle[axis] + angles_dps[axis] * dtTotal * DEG2RAD;
        
        // Wrap into [-pi, pi)
        localAngle[axis] = wrapPi(localAngle[axis]);
    }
}

// -------------------------------------------------------------------------
// ACCEL UPDATE ~650us
// -------------------------------------------------------------------------
void odometryIMUAccelUpdate(uint8_t numSamples, float dtPerSample, float dtTotal)
{
    // Called when new accel data are available in:
    //   extern volatile int16_t rawAccelMeasurements[3]; -> average raw readings from sensor since last FIFO pull
    //   extern volatile int16_t rawAccelSum[3]; -> sum of raw readings from sensor since last FIFO pull
    
    if (!initDone && !initAccelEstimates) {
        return;
    }
    /*
    static uint32_t i = 0;
    if (i % 20 == 0) {
        char measurementStr[70];
        snprintf(measurementStr, 70, "Accelerometer [g]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accel_g[0], accel_g[1], accel_g[2]);
        putsUART1Str(measurementStr);
    }
    i++;
    */
    
    float accel_raw[3];
    imuCalibrateAccelMeasurementsFloat(rawFifoAccelMeasurements, accel_raw);
    
    // Calculate roll and pitch estimates from raw values
    float roll = -atan2f(accel_raw[X], accel_raw[Z]); // atan(x/z)
    float pitch = atan2f(-accel_raw[Y], sqrtf(accel_raw[X]*accel_raw[X] + accel_raw[Z]*accel_raw[Z])); // atan(y/sqrt(x^2+z^2))
    
    // Initialize estimates
    if (initAccelEstimates) {
        localAngle[PITCH] = pitch;
        mouseAngle[PITCH] = pitch;
        localAngle[ROLL] = roll;
        mouseAngle[ROLL] = roll;
        
        initAccelEstimates = false;
        initDone = !initAccelEstimates && !initMagEstimates;
        return;
    }
    
    mouseAccelPitch = pitch;
    mouseAccelRoll = roll;
    
    // Fuse with gyroscope readings (complementary filter)
    mouseAngle[PITCH] = fuseAngle(localAngle[PITCH], pitch, dtTotal, CF_TAU_PITCHROLL);
    mouseAngle[ROLL]  = fuseAngle(localAngle[ROLL],  roll,  dtTotal, CF_TAU_PITCHROLL);
    
    // Yaw rate will be improved by magnetometer and wheel encoders at a slower rate
    mouseAngle[YAW] = localAngle[YAW];
    
    // Zero-Velocity Update (ZUPT)
    
    // Scale raw accelerometer to g's:
    
    /*
    if(robotIsStationary()) {
        mouseVelocity[0] = 0.0f;
        mouseVelocity[1] = 0.0f;
        mouseVelocity[2] = 0.0f;
        
        // Early return: no need to integrate velocity and position when stationary
        return;
    }
    
    float accel_g[3];
    imuScaleAccelMeasurementsFloat(accel_raw, accel_g);
    
    // GRAVITY COMPENSATION  (all math done in g's)
    //   body->frame gravity from current roll & pitch:
    //        g_b = [-sin(roll),  sin(sin)·cos(pitch),  cos(roll)·cos(pitch)]
    float rollSin = sinf(mouseAngle[ROLL]);
    float rollCos = cosf(mouseAngle[ROLL]);
    float pitchSin = sinf(mouseAngle[PITCH]);
    float pitchCos = cosf(mouseAngle[PITCH]);

    float g_bx = -pitchSin;
    float g_by =  rollSin * pitchCos;
    //float g_bz =  rollCos * pitchCos;

    // specific force (still in g's)
    float f_gx = accel_g[X] - g_bx;
    float f_gy = accel_g[Y] - g_by;
    
    char buf[40];
    snprintf(buf, sizeof(buf), "f_gx=%f  f_gy=%f\r\n", f_gx, f_gy);
    putsUART1Str(buf);

    // Z is not used for 2-D odometry, but you could keep it:
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
    mouseVelocity[0] += accelGlobalX * dtTotal;  // vx
    mouseVelocity[1] += accelGlobalY * dtTotal;  // vy

    // Integrate velocity to get position in global frame (x,y):
    mousePosition[0] += mouseVelocity[0] * dtTotal;    // x
    mousePosition[1] += mouseVelocity[1] * dtTotal;    // y
    */
}


// -------------------------------------------------------------------------
// MAG UPDATE ~440us
// -------------------------------------------------------------------------
void odometryIMUMagUpdate(float dtTotal) {
    if (!initDone && !initMagEstimates) {
        return;
    }
    
    float scaledMagMeasurements[3];
    
    // Get calibrated magnetometer readings
    imuCalibrateMagMeasurementsFloat(rawFifoMagMeasurements, scaledMagMeasurements);
    
    // Get magnetometer heading using pitch and roll estimates
    //float magYaw = magnetometerToHeading(scaledMagMeasurements);
    float magYaw = magnetometerToTiltCompensatedHeading(scaledMagMeasurements, mouseAngle[PITCH], mouseAngle[ROLL]);
    
    static int   magInitCount = 0;
    static float magInitSum   = 0.0f;

    // Initialize estimates with an N sample average
    if (initMagEstimates)
    {
        // accumulate
        magInitSum  += magYaw;
        magInitCount++;

        if (magInitCount >= MAG_INIT_SAMPLES)
        {
            // compute average
            float avgYaw = magInitSum / (float)MAG_INIT_SAMPLES;

            // set initial state
            localAngle[YAW] = avgYaw;
            mouseAngle[YAW] = avgYaw;

            // done initializing
            initMagEstimates = false;
            initDone = !initAccelEstimates && !initMagEstimates;

            // (optional) reset counters in case you ever re-init
            magInitCount = 0;
            magInitSum   = 0.0f;
        }

        // on startup we do not run the rest of the algorithm until we have our average
        return;
    }
    
    mouseMagYaw = magYaw;
    
    // Fuse heading with estimated yaw of gyroscope
    if (fabsf(lastYawRate) < FAST_TURN_RAD_S) {      // ignore while fast turning
        localAngle[YAW] = fuseAngle(localAngle[YAW], magYaw, dtTotal, CF_TAU_YAW_MAG);
        mouseAngle[YAW] = localAngle[YAW];
    }
    
    // Update motor encoders
    updateEncoderVelocities();
    odometryEncoderUpdate();
}

// -------------------------------------------------------------------------
// ENCODER UPDATE ~220us
// -------------------------------------------------------------------------
void odometryEncoderUpdate(void)
{
    static float yawAtLastEncUpdate = 0.0f;
    uint64_t currentTimeUs = getTimeInUs();

    if (!encInit) {
        lastEncoderUpdateTimeUs = currentTimeUs;
        yawAtLastEncUpdate = mouseAngle[YAW];
        encInit = true;
        return;
    }
    
    uint64_t deltaUs = currentTimeUs - lastEncoderUpdateTimeUs;
    lastEncoderUpdateTimeUs = currentTimeUs;

    
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
    
    localAngle[YAW] = fuseAngle(localAngle[YAW], encoderYaw, dt, CF_TAU_YAW_ENC);
    mouseAngle[YAW] = localAngle[YAW];
    
    /* save for the *next* cycle */
    yawAtLastEncUpdate = mouseAngle[YAW];
    
    
    //mouseVelocity[Y] = linearVelocityBody;
    //mousePosition[Y] += mouseVelocity[Y] * dt;
    
    /* ---------- NEW: transform encoder velocity to world frame ---------- */
    float yaw = mouseAngle[YAW];      // current CW yaw
    float cosYaw = cosf(yaw);
    float sinYaw = sinf(yaw);

    /* body-frame vel: +Y = forward, +X = right  (v_bx = 0 here)   */
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
    //putsUART1Str(elapsedStr);
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