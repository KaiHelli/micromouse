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

/** Encoder reference (um) marking the start of the current maze cell. */
static int32_t currentCellStartMicrometers;

/* -------------------------------------------------------------------------- */
/* Helper functions                                                           */
/* -------------------------------------------------------------------------- */

static void waitForSensorUpdate() {
    __delay_ms(16);
}

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
    centerMouseInCell();        // user-driven centre-of-cell alignment
    resetControlErrors();       // zero all PID integrators, etc.
    disableWallsControl();      // start clean, no wall-following
    setStartingPosition();      // record current encoder as cell start
}

/**
 * @brief Return the distance travelled inside the present cell, in meters.
 */
static int32_t currentCellShift(void)
{
    return ((int32_t) getEncoderAverageDistanceUm() - currentCellStartMicrometers);
}

/**
 * @brief Mark the entry of a new cell, applying any longitudinal correction.
 */
static void enteredNextCell(void)
{
    int32_t frontWallCorrection;

    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm() - MIDDLE_MAZE_DISTANCE_UM;
    
    if (sensorIsWallFront()) {
        frontWallCorrection = (int32_t) getRobotDistanceUm(SENSOR_CENTER) - (int32_t) (0.5f * CELL_DIMENSION_UM);
        currentCellStartMicrometers += frontWallCorrection;
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void setStartingPosition(void)
{
    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm() - MIDDLE_MAZE_DISTANCE_UM;
}

int32_t requiredMicrometersToSpeed(float speed)
{
    float acceleration;
    float currentSpeed = getIdealLinearSpeed();

    acceleration = (currentSpeed > speed) ? -getLinearDeceleration() : getLinearAcceleration();

    return (int32_t) ((speed * speed - currentSpeed * currentSpeed) /
                     (2 * acceleration) * MICROMETERS_PER_METER);
}

float requiredTimeToSpeed(float speed)
{
    float acceleration;
    float targetSpeed = getTargetLinearSpeed();

    acceleration = (targetSpeed > speed) ? -getLinearDeceleration() : getLinearAcceleration();

    return (speed - targetSpeed) / acceleration;
}

uint32_t requiredTicksToSpeed(float speed, float hz)
{
    float seconds = requiredTimeToSpeed(speed);
    return (uint32_t)(seconds * hz);
}

/**
 * @brief Drive a straight segment and reach the target speed at its end.
 *        Positive @p distance moves forward, negative moves backward.
 */
void targetStraight(int32_t startMicrometers, float distance, float endSpeed)
{
    int32_t targetDistance = startMicrometers + (int32_t) (distance);

    /* No rotation while driving straight. */
    setIdealAngularSpeed(0.0f);

    if (distance > 0) {
        /* Accelerate */
        setTargetLinearSpeed(getMaxLinearSpeed());
        while (getEncoderAverageDistanceUm() < targetDistance - requiredMicrometersToSpeed(endSpeed))
            ;
    } else {
        setTargetLinearSpeed(-getMaxLinearSpeed());
        while (getEncoderAverageDistanceUm() > targetDistance - requiredMicrometersToSpeed(endSpeed))
            ;
    }

    /* Decelerate / settle to requested endSpeed */
    setTargetLinearSpeed(endSpeed);

    if (endSpeed == 0.0f) {
        while (getIdealLinearSpeed() != 0.0f) ;
    } else {
        while ((distance > 0 && getEncoderAverageDistanceUm() < targetDistance) ||
               (distance < 0 && getEncoderAverageDistanceUm() > targetDistance))
            ;
    }
}

/* -------------------------------------------------------------------------- */
/* Wall-alignment helpers                                                     */
/* -------------------------------------------------------------------------- */

void squareUpByWiggle(float initStepDeg, uint8_t maxIter, float force)
{
    //uprintf("\r\n[SUWB] Start: step=%.2f deg, maxIter=%u, force=%.2f\r\n", initStepDeg, maxIter, force);

    float    yaw        = 0.0f;      /* absolute heading from function entry (deg) */
    float    step       = initStepDeg;
    uint32_t bestDist   = getRobotDistanceUm(SENSOR_CENTER);
    float    bestYawDeg = yaw;

    //uprintf("[SUWB] Initial: yaw=%.2f deg, dist=%lu um\r\n", yaw, bestDist);

    for (uint8_t k = 0; k < maxIter; ++k) {
        /* ?? 1. measure the distance at the current heading ??????????? */
        waitForSensorUpdate();                           /* make sure reading is fresh */
        uint32_t dCenter = getRobotDistanceUm(SENSOR_CENTER);

        /* ?? 2. probe +step ????????????????????????????????????????????? */
        inplaceTurn( step * DEG2RAD, force);
        yaw += step;
        waitForSensorUpdate();
        uint32_t dPlus = getRobotDistanceUm(SENSOR_CENTER);

        /* ?? 3. probe ?step (two half?turns: back to centre, then ?step) ? */
        inplaceTurn(-2.0f * step * DEG2RAD, force);             /* back to centre           */
        yaw -= 2.0f * step;

        waitForSensorUpdate();
        uint32_t dMinus = getRobotDistanceUm(SENSOR_CENTER);

        /* ?? 4. decide which of the three positions is best ????????????? */
        float    offsetDeg   = step;   /* +step, ?step or 0 for centre */
        uint32_t iterBest    = dCenter;

        if (dPlus < iterBest) { iterBest = dPlus;   offsetDeg = 2.0f * step; }
        if (dMinus < iterBest){ iterBest = dMinus;  offsetDeg = 0.0f; }

        /* ?? 5. move to the best position of this iteration (if needed) ? */
        if (offsetDeg != 0.0f) {
            inplaceTurn(offsetDeg * DEG2RAD, force);
            yaw += offsetDeg;
        }

        /* ?? 6. remember the global best so far ????????????????????????? */
        if (iterBest < bestDist) { bestDist = iterBest; bestYawDeg = yaw; }

        //uprintf("[SUWB] k=%u | step=%.3f | yaw=%.3f | d0=%lu | d+=%lu | d-=%lu | best=%lu @ %.3f\r\n", k, step, yaw, dCenter, dPlus, dMinus, bestDist, bestYawDeg);

        /* ?? 7. shrink the search window ???????????????????????????????? */
        step *= 0.5f;
        if (step < 0.05f) {
            //uprintf("[SUWB] Step below 0.05 deg -> done.\r\n");
            break;
        }
    }

    /* ?? 8. return to the absolute best orientation found ?????????????? */
    float deltaDeg = bestYawDeg - yaw;
    inplaceTurn(deltaDeg * DEG2RAD, force);

    //uprintf("[SUWB] Finish: bestYawDeg=%.3f?deg, bestDist=%lu?um ? returning.\r\n", bestYawDeg, bestDist);
}

void keepFrontWallDistance(float distance)
{
    int i;
    float diff, frontWallDistance;

    if (!sensorIsWallFront())
        return;
    
    squareUpByWiggle(10.0, 10, getMaxForce());

    float maxForce = getMaxForce();
    setMaxForce(0.04f);
    
    while (true) {
        sideSensorsCloseControl(false);
        sideSensorsFarControl(false);

        frontWallDistance = 0.0f;
        for (i = 0; i < 20; ++i) {
            frontWallDistance += (float) (getRobotDistanceUm(SENSOR_CENTER));
            waitForSensorUpdate();
        }
        frontWallDistance /= 20;
        diff = frontWallDistance - distance;
        
        uprintf("diff: %.5f\r\n", diff);
        
        if (fabsf(diff) < KEEP_FRONT_DISTANCE_TOLERANCE_UM)
            break;

        targetStraight(getEncoderAverageDistanceUm(), diff, 0.0f);
    }

    setMaxForce(maxForce);

    disableWallsControl();
    resetControlAll();
}

/* -------------------------------------------------------------------------- */
/* Basic stopping helpers                                                     */
/* -------------------------------------------------------------------------- */

void stopEnd(void)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraight(currentCellStartMicrometers, CELL_DIMENSION_UM, 0.0f);

    disableWallsControl();
    resetControlErrors();
    enteredNextCell();
}

void stopHeadFrontWall(void)
{
    const float distance = CELL_DIMENSION_UM - WALL_WIDTH_UM / 2.0f - MOUSE_HEAD_MM * MICROMETERS_PER_MILLIMETER;

    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraight(currentCellStartMicrometers, distance, 0.0f);

    disableWallsControl();
    resetControlErrors();
}

void stopMiddle(void)
{
    const float distance = CELL_DIMENSION_UM / 2.0f;

    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraight(currentCellStartMicrometers, distance, 0.0f);

    disableWallsControl();
    resetControlErrors();
}

/* -------------------------------------------------------------------------- */
/* In-place rotations                                                         */
/* -------------------------------------------------------------------------- */

void inplaceTurnDeg(float degrees, float force) {
    inplaceTurn(degrees * DEG2RAD, force);
}

void inplaceTurn(float radians, float force)
{
    uint64_t startTime, curTime;
    float angularAcceleration, t, duration, transition, arc, transitionAngle, maxAngularVelocity, angularVelocity, factor;
    
    int signDir = sign(radians);
    radians = fabsf(radians);

    /* Compute trapezoidal angular profile. */
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

/* -------------------------------------------------------------------------- */
/* Composite high-level moves                                                 */
/* -------------------------------------------------------------------------- */

/**
 * Drive one whole cell forward (centre -> centre).
 */
static void forwardOneCell(float cruiseSpeed, float endSpeed)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    setMaxLinearSpeed(cruiseSpeed);

    /* centre -> centre is 1.5 × CELL when startMicrometers is 0.5?cell behind */
    targetStraight(currentCellStartMicrometers, CELL_DIMENSION_UM + MIDDLE_MAZE_DISTANCE_UM, endSpeed);

    enteredNextCell();
}

/**
 * Perform a 90° pivot in place while sitting in the cell centre.  No
 * longitudinal motion is required because we already have clearance front and
 * back (1/2-cell each way).
 */
void pivot90(Movement_t dir, float force)
{
    disableWallsControl();

    if (dir == MOVE_LEFT)
        inplaceTurn( -M_PI / 2.0f, force);
    else
        inplaceTurn( M_PI / 2.0f, force);
}


/**
 * Perform a 180° turn (dead-end recovery) while staying centred.
 */
void pivot180(float force)
{   
    disableWallsControl();
    
    int16_t dirSign = (int16_t) (rand() % 2) * 2 - 1;      // random left/right pivot

    inplaceTurn(dirSign * M_PI, force);
}

/**
 * @brief  Move forward *n* cells (centre -> centre).
 *
 * The function accelerates once, cruises through the middle cells and decelerates
 * so that it reaches @p endSpeed exactly at the centre of the last cell.
 */
void moveForwardCenterCells(uint8_t nCells, float cruiseSpeed, float endSpeed)
{
    if (nCells == 0) return;

    for (uint8_t k = 1; k < nCells; ++k)
        forwardOneCell(cruiseSpeed, cruiseSpeed);

    forwardOneCell(cruiseSpeed, endSpeed);
}

/**
 * @brief  One-cell step forward.
 */
void moveForwardCenter(float cruiseSpeed, float endSpeed)
{
    forwardOneCell(cruiseSpeed, endSpeed);
}

/**
 * @brief  90° left turn, centre-pivot.
 */
void turnLeftCenter(float force)
{
    pivot90(MOVE_LEFT, force);
}

/**
 * @brief  90° right turn, centre-pivot.
 */
void turnRightCenter(float force)
{
    pivot90(MOVE_RIGHT, force);
}

/**
 * @brief  Dead-end handling: square-up, 180° pivot, step back one cell.
 */
void escapeDeadEnd(float force)
{
    if (getRobotDistanceUm(SENSOR_CENTER) < CELL_DIMENSION_UM)
        keepFrontWallDistance(MIDDLE_MAZE_DISTANCE_UM);

    pivot180(force);
    
    setStartingPosition();
    currentCellStartMicrometers -= (int32_t) SHIFT_AFTER_180_DEG_TURN_UM;
}