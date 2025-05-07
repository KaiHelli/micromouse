/* Source code inspired by:
 *
 * Movements and Kinematics: Bulebule
 * - https://github.com/Bulebots/bulebule
 * - https://github.com/Bulebots/mmlib
 */ 

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "move.h"
#include "IOConfig.h"
#include "odometry.h"
#include "timers.h"
#include "fastPID.h"
#include "uart.h"
#include "constants.h"
#include "sensors.h"
#include "motors.h"
#include "motorEncoders.h"
#include "mouseController.h"
#include "globalTimers.h"

// Encoder reference (um) marking the start of the current maze cell.
static int32_t currentCellStartMicrometers;

/*--------------------------------------------------------------------
 * Helper functions
 *------------------------------------------------------------------*/
static int sign(float number)
{
	return (int)(number > 0) - (int)(number < 0);
}

#define SENSOR_OFFSET_THRESHOLD_UM (1 * MICROMETERS_PER_MILLIMETER)

void centerMouseInCell() {
    uprintf("Center mouse in cell.\r\n");
    int32_t lrOffset = 0;
    int32_t frOffset = 0;
    do {
        uint32_t left  = getRobotDistanceUm(SENSOR_LEFT);
        uint32_t right = getRobotDistanceUm(SENSOR_RIGHT);
        uint32_t front = getRobotDistanceUm(SENSOR_CENTER);

        lrOffset = (int32_t) right - (int32_t) left;
        frOffset = (int32_t) front - (int32_t) MIDDLE_MAZE_DISTANCE_UM;
        uprintf("LR Offset: %ld | FR Offset: %ld | L: %lu | F: %lu | R: %lu\r\n", lrOffset, frOffset, left, front, right);
        
        __delay_ms(500);
    } while (labs(lrOffset) > SENSOR_OFFSET_THRESHOLD_UM || labs(frOffset) > SENSOR_OFFSET_THRESHOLD_UM);
    
    uprintf("Centering done. Don't move anymore.\r\n");
}

void calibrateStartPosition(void)
{
    //centerMouseInCell();        // user-driven centre-of-cell alignment

    pivot90(MOVE_LEFT, getMaxForce());
    keepFrontWallDistance(MIDDLE_MAZE_DISTANCE_UM);
    pivot90(MOVE_RIGHT, getMaxForce());
    keepFrontWallDistance(MIDDLE_MAZE_DISTANCE_UM);
    
    resetControlErrors();       // zero all PID integrators, etc.
    disableWallsControl();      // start clean, no wall-following
    setStartingPosition();      // record current encoder as cell start
}

static int32_t currentCellShift(void)
{
    return ((int32_t) getEncoderAverageDistanceUm() - currentCellStartMicrometers);
}

static void resetOdometryToStart(void)
{
    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm();
}

static void resetOdometryToCenter(void)
{
    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm() - MIDDLE_MAZE_DISTANCE_UM;
}

static void correctNearCellShift(bool isCenter)
{
    int32_t frontWallCorrection;

    const int32_t expectedDistance = isCenter ? MIDDLE_MAZE_DISTANCE_UM : (CELL_DIMENSION_UM - WALL_WIDTH_UM/2);
    
    if (sensorIsWallFront()) {
        int32_t actualDistance = (int32_t) medianRobotDistanceUm(SENSOR_CENTER, 5);
        frontWallCorrection = actualDistance - expectedDistance;
        currentCellStartMicrometers += frontWallCorrection;
        
        //uprintf("Cell Start: %ld, Front Wall: %ld, Error: %ld\r\n", currentCellStartMicrometers, actualDistance, frontWallCorrection);
    }
}

static void correctFarCellShift(void) 
{
    int32_t frontWallCorrection;

    if (sensorIsWallFarFront()) {
        int32_t actualDistance = (int32_t) medianRobotDistanceUm(SENSOR_CENTER, 5);
        frontWallCorrection = actualDistance - (CELL_DIMENSION_UM + MIDDLE_MAZE_DISTANCE_UM + CORRECT_FRONT_DISTANCE_BIAS_UM);
        currentCellStartMicrometers += frontWallCorrection;
        
        //uprintf("Cell Start: %ld, Front Wall: %ld, Error: %ld\r\n", currentCellStartMicrometers, actualDistance, frontWallCorrection);
    }
}

/*--------------------------------------------------------------------
 * Public API
 *------------------------------------------------------------------*/

void setStartingPosition(void)
{
    resetOdometryToCenter();
    correctNearCellShift(true);
}

int32_t requiredMicrometersToSpeed(float speed)
{
    float acceleration;
    float currentSpeed = getIdealLinearSpeed();

    acceleration = (currentSpeed > speed) ? -getLinearDeceleration() : getLinearAcceleration();

    return (int32_t) ((speed * speed - currentSpeed * currentSpeed) /
                     (2 * acceleration) * MICROMETERS_PER_METER);
}

float requiredTimeToSpeed(float currentSpeed, float targetSpeed)
{
    float acceleration;

    acceleration = (currentSpeed > targetSpeed) ? -getLinearDeceleration() : getLinearAcceleration();

    return (targetSpeed - currentSpeed) / acceleration;
}

void targetStraightEncoders(int32_t startMicrometers, int32_t distance, float endSpeed)
{
    int32_t targetDistance = startMicrometers + distance;

    // No rotation while driving straight.
    setIdealAngularSpeed(0.0f);

    if (distance > 0) {
        // Accelerate
        setTargetLinearSpeed(getMaxLinearSpeed());
        while ((int32_t) getEncoderAverageDistanceUm() < targetDistance - requiredMicrometersToSpeed(endSpeed))
            //uprintf("%ld, %ld, %ld\r\n", (int32_t) getEncoderAverageDistanceUm(), targetDistance, requiredMicrometersToSpeed(endSpeed));
            ;
    } else {
        setTargetLinearSpeed(-getMaxLinearSpeed());
        while ((int32_t) getEncoderAverageDistanceUm() > targetDistance - requiredMicrometersToSpeed(endSpeed))
            ;
    }

    // Decelerate / settle to requested endSpeed
    setTargetLinearSpeed(endSpeed);

    if (endSpeed == 0.0f) {
        while (getIdealLinearSpeed() != 0.0f) ;
    } else {
        while ((distance > 0 && getEncoderAverageDistanceUm() < targetDistance) ||
               (distance < 0 && getEncoderAverageDistanceUm() > targetDistance))
            ;
    }
}

void targetStraightSpeed(int32_t startMicrometers, int32_t distance, float endSpeed)
{
    int32_t orig_distance = distance;
    int32_t currentPos    = (int32_t)getEncoderAverageDistanceUm();
    //uprintf("TS: start=%ld orig_dist=%ld currentPos=%ld\r\n", startMicrometers, orig_distance, currentPos);
    
    // Correct the distance by where we are now
    distance = startMicrometers + orig_distance - currentPos;
    //uprintf("TS: corrected distance_um=%ld\r\n", distance);
    
    // Get direction and distance in meters
    bool  forward = (distance >= 0);
    float sign    = forward ? +1.0f : -1.0f;
    float dist_m  = (float) labs(distance) / MICROMETERS_PER_METER;
    
    //uprintf("TS: forward=%d dist_m=%.4f m\r\n", forward, dist_m);
    
    // Get speeds
    float v0   = fabsf(getTargetLinearSpeed());
    float vend = fabsf(endSpeed);
    float vmax = getMaxLinearSpeed();
    float acc    = getLinearAcceleration();
    float dec    = getLinearDeceleration();
    
    //uprintf("TS: v0=%.3f vend=%.3f vmax=%.3f acc=%.3f dec=%.3f\r\n", v0, vend, vmax, acc, dec);
    
    // Compute accel / decel times & distances at full cruise
    float t_acc = fmaxf(0.0f, (vmax - v0) / acc);
    float d_acc = (v0 + vmax) * 0.5f * t_acc;

    float t_dec = fmaxf(0.0f, (vmax - vend) / dec);
    float d_dec = (vmax + vend) * 0.5f * t_dec;
    
    //uprintf("TS: t_acc=%.6f d_acc=%.6f t_dec=%.6f d_dec=%.6f\r\n", t_acc, d_acc, t_dec, d_dec);
    
    // Check if distance fits a full trapezoid
    float t_cruise;
    float peak_v = vmax;
    if (d_acc + d_dec > dist_m) {
        // too short -> triangular: solve for peak_v
        peak_v = sqrtf((2.0f*acc*dec*dist_m + dec*v0*v0 + acc*vend*vend) / (acc + dec));
        t_acc = (peak_v - v0) / acc;
        t_dec = (peak_v - vend) / dec;
        t_cruise = 0.0f;
        
        
        //uprintf("TS: PROFILE=TRI peak_v=%.3f t_acc=%.6f t_dec=%.6f\r\n", peak_v, t_acc, t_dec);
    } else {
        // there's some cruise plateau
        t_cruise = (dist_m - (d_acc + d_dec)) / vmax;
        
        //uprintf("TS: PROFILE=TRAP t_cruise=%.6f\r\n", t_cruise);
    }
    
    // Convert to ?s for busy-wait loops
    uint64_t usec_acc    = (uint64_t)(t_acc    * MICROSECONDS_PER_SECOND);
    uint64_t usec_cruise = (uint64_t)(t_cruise * MICROSECONDS_PER_SECOND);
    uint64_t usec_dec    = (uint64_t)(t_dec    * MICROSECONDS_PER_SECOND);
    
    //uprintf("TS: usec_acc=%llu usec_cruise=%llu usec_dec=%llu\r\n", usec_acc, usec_cruise, usec_dec);
    
    uint64_t t0 = getTimeInUs();
    
    //uprintf("TS: PHASE1: accel to %.3f m/s (until %lluus)\r\n", sign * peak_v, usec_acc);
    // Accelerate to peak_v
    setTargetLinearSpeed(sign * peak_v);
    while (getTimeInUs() - t0 < usec_acc) { /* spin */ }

    //uprintf("TS: END PH1: dt=%lluus enc=%ld\r\n", getTimeInUs() - t0, (int32_t)getEncoderAverageDistanceUm());
    
    // Cruise at vmax - if reached
    if (usec_cruise > 0) {
        //uprintf("TS: PHASE2: cruise at %.3f m/s (until %lluus)\r\n", sign * vmax, usec_acc + usec_cruise);
        setTargetLinearSpeed(sign * vmax);
        while (getTimeInUs() - t0 < usec_acc + usec_cruise) { /* spin */ }
        //uprintf("TS: END PH2: dt=%lluus enc=%ld\r\n", getTimeInUs() - t0, (int32_t)getEncoderAverageDistanceUm());
    }
    
    // Decelerate to endSpeed
    //uprintf("TS: PHASE3: decel to %.3f m/s (until %lluus)\r\n", sign * vend, usec_acc + usec_cruise + usec_dec);
    setTargetLinearSpeed(sign * vend);
    while (getTimeInUs() - t0 < usec_acc + usec_cruise + usec_dec) { /* spin */ }
    
    //uprintf("TS: END PH3: dt=%lluus enc=%ld targetPos=%ld\r\n", getTimeInUs() - t0, (int32_t)getEncoderAverageDistanceUm(), startMicrometers + orig_distance);
}
    

/*--------------------------------------------------------------------
 * Wall-alignment helpers
 *------------------------------------------------------------------*/

void squareUp(float probeAngleDeg, float force, uint32_t numReadings)
{   
    const float alpha    = probeAngleDeg * DEG2RAD;
    const float minError = 0.1f * DEG2RAD;  // stop when <0.1°
    const int   maxIter  = 3;               // at most a few passes

    for (int i = 0; i < maxIter; ++i) {
        // Turn +alpha and measure
        inplaceTurn(+alpha, force);
        float dL = (float)medianRobotDistanceUm(SENSOR_CENTER, numReadings);

        // Turn ?2*alpha (to -alpha) and measure
        inplaceTurn(-2 * alpha, force);
        float dR = (float)medianRobotDistanceUm(SENSOR_CENTER, numReadings);

        // Compute angle error: theta = 1/2·asin((dL-dR)/(dL+dR))
        float sum = dL + dR;
        if (sum <= 0.0f) {
            // sensor error guard ? return to original heading and bail out
            inplaceTurn(+alpha, force);
            break;
        }

        float ratio = (dL - dR) / sum;
        // clamp to [-1,1] to avoid NaN
        if      (ratio >  1.0f) ratio =  1.0f;
        else if (ratio < -1.0f) ratio = -1.0f;

        float theta = 0.5f * asinf(ratio);

        // If already aligned within tolerance, return to zero and finish
        if (fabsf(theta) < minError) {
            inplaceTurn(+alpha, force);
            break;
        }

        // Apply the correcting turn (bring sensor perpendicular)
        // we were at -alpha, so turning by alpha?theta brings us to ?theta,
        // then the next loop?s +alpha will move us to +(alpha-theta)
        inplaceTurn(alpha - theta, force);
    }
}

void keepFrontWallDistance(int32_t distance)
{
    int32_t diff;
    uint32_t frontWallDistance;

    if (!sensorIsWallFront())
        return;
    
    squareUp(20.0f, 0.2f, 11);

    float maxForce = getMaxForce();
    setMaxForce(0.04f);
    
    while (true) {
        sideSensorsCloseControl(false);
        sideSensorsFarControl(false);

        frontWallDistance = medianRobotDistanceUm(SENSOR_CENTER, 11);
        diff = frontWallDistance - distance;
        
        //uprintf("diff: %.5f\r\n", diff);
        
        if (abs(diff) < KEEP_FRONT_DISTANCE_TOLERANCE_UM)
            break;

        targetStraightEncoders(getEncoderAverageDistanceUm(), diff, 0.0f);
    }

    setMaxForce(maxForce);

    disableWallsControl();
    resetControlAll();
}

/*--------------------------------------------------------------------
 * Basic stopping helpers
 *------------------------------------------------------------------*/

void stopEnd(void)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraightEncoders(currentCellStartMicrometers, CELL_DIMENSION_UM, 0.0f);

    disableWallsControl();
    resetControlErrors();
    
    resetOdometryToStart();
    correctNearCellShift(false);
}

void stopHeadFrontWall(void)
{
    const int32_t distance = (int32_t) (CELL_DIMENSION_UM - WALL_WIDTH_UM / 2.0f - MOUSE_HEAD_MM * MICROMETERS_PER_MILLIMETER);

    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraightEncoders(currentCellStartMicrometers, distance, 0.0f);

    disableWallsControl();
    resetControlErrors();
}

void stopMiddle(void)
{
    const int32_t distance = (int32_t) (CELL_DIMENSION_UM / 2.0f);

    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraightEncoders(currentCellStartMicrometers, distance, 0.0f);

    disableWallsControl();
    resetControlErrors();
}


/*--------------------------------------------------------------------
 * In-place rotations
 *------------------------------------------------------------------*/

void inplaceTurnDeg(float degrees, float force) {
    inplaceTurn(degrees * DEG2RAD, force);
}

void inplaceTurn(float radians, float force)
{
    uint64_t startTime, curTime;
    float angularAcceleration, t, duration, transition, arc, transitionAngle, maxAngularVelocity, angularVelocity, factor;
    
    int signDir = sign(radians);
    radians = fabsf(radians);

    // Compute trapezoidal / sinusoidal angular profile.
    angularAcceleration = force * MOUSE_WHEEL_SEPARATION_MM / (MILLIMETERS_PER_METER * MOUSE_MOMENT_OF_INERTIA_KGM2);
    maxAngularVelocity = sqrtf(radians / 2.0f * angularAcceleration);
    if (maxAngularVelocity > MOUSE_MAX_ANGULAR_VELOCITY_RADPS)
        maxAngularVelocity = MOUSE_MAX_ANGULAR_VELOCITY_RADPS;
    
    duration = maxAngularVelocity / angularAcceleration * M_PI;
    transitionAngle = duration * maxAngularVelocity / M_PI;
    arc = (radians - 2.0f * transitionAngle) / maxAngularVelocity;
    transition = duration / 2.0f;
    maxAngularVelocity *= signDir; 
    
    setTargetLinearSpeed(getIdealLinearSpeed());
    disableWallsControl();

    startTime = getTimeInUs();
    while (true) {
        curTime = getTimeInUs();
        t = (float) (curTime - startTime) / MICROSECONDS_PER_SECOND;
        if (t >= 2 * transition + arc)
            break;

        angularVelocity = maxAngularVelocity;
        if (t < transition) {
            factor = t / transition;
            angularVelocity *= sinf(factor * M_PI / 2.0f);
        } else if (t >= transition + arc) {
			factor = (t - arc) / transition;
            angularVelocity *= sinf(factor * M_PI / 2.0f);
        }
        
        setIdealAngularSpeed(angularVelocity);
    }
    
    setIdealAngularSpeed(0.0f);
}


/*--------------------------------------------------------------------
 * Composite high-level moves
 *------------------------------------------------------------------*/

static void forwardOneCell(float cruiseSpeed, float endSpeed, bool accountForShift)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    setMaxLinearSpeed(cruiseSpeed);

    int32_t halfDistance = CELL_DIMENSION_UM / 2;
    
    if (accountForShift) {
        halfDistance += SHIFT_DISTANCE_UM / 2;
    }
    
    targetStraightEncoders(currentCellStartMicrometers, halfDistance + MIDDLE_MAZE_DISTANCE_UM, cruiseSpeed);

    //uprintf("1: ccsm: %ld, ce: %ld, hd: %ld\r\n", currentCellStartMicrometers, (int32_t) getEncoderAverageDistanceUm(), halfDistance);
    
    resetOdometryToStart();
    correctNearCellShift(false);
    
    targetStraightEncoders(currentCellStartMicrometers, halfDistance, endSpeed);

    //uprintf("2: ccsm: %ld, ce: %ld, hd: %ld\r\n", currentCellStartMicrometers, (int32_t) getEncoderAverageDistanceUm(), halfDistance);
}

void pivot90(Movement_t dir, float force)
{
    disableWallsControl();

    if (dir == MOVE_LEFT)
        inplaceTurn( -M_PI / 2.0f, force);
    else
        inplaceTurn( M_PI / 2.0f, force);
}

void pivot180(float force)
{   
    disableWallsControl();
    
    int16_t dirSign = (int16_t) (rand() % 2) * 2 - 1;      // random left/right pivot

    inplaceTurn(dirSign * M_PI, force);
}

void moveForwardCenterCells(uint8_t nCells, float cruiseSpeed, float endSpeed)
{
    if (nCells == 0) return;

    for (uint8_t k = 1; k < nCells; ++k)
        forwardOneCell(cruiseSpeed, cruiseSpeed, false);

    forwardOneCell(cruiseSpeed, endSpeed, true);
}

void moveForwardCenter(float cruiseSpeed, float endSpeed)
{
    forwardOneCell(cruiseSpeed, endSpeed, true);
}

void turnLeftCenter(float force)
{
    pivot90(MOVE_LEFT, force);
    
    resetOdometryToCenter();
}

void turnRightCenter(float force)
{
    pivot90(MOVE_RIGHT, force);
    
    resetOdometryToCenter();
}

void turnAroundCenter(float force)
{
    pivot180(force);
    
    resetOdometryToCenter();
    currentCellStartMicrometers += (int32_t) SHIFT_AFTER_180_DEG_TURN_UM;
}

void escapeDeadEnd(float force)
{
    keepFrontWallDistance(MIDDLE_MAZE_DISTANCE_UM);

    pivot180(force);
    
    resetOdometryToCenter();
    currentCellStartMicrometers += (int32_t) SHIFT_AFTER_180_DEG_TURN_UM;
}