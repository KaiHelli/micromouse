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

void sleepTicks(uint16_t ticks) {
    //TODO - not exactly right, due to the loop overhead... but fine for now
    for (uint16_t i = 0; i < ticks; i++) {
        __delay_ms(10);
    }
}

int sign(float number)
{
	return (int)(number > 0) - (int)(number < 0);
}

#define SENSOR_OFFSET_THRESHOLD 1000 // TODO

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
        uprintf("LR Offset: %ld | FR Offset: %ld\r\n", lrOffset, frOffset);
        uprintf("L: %u F: %u R: %u\r\n", left, front, right);
        
        __delay_ms(500);
    } while (labs(lrOffset) > SENSOR_OFFSET_THRESHOLD || labs(frOffset) > SENSOR_OFFSET_THRESHOLD);
    
    uprintf("Centering done. Don't move anymore.\r\n");
}

void calibrateStartPosition(void)
{
    centerMouseInCell();        // user?driven centre?of?cell alignment
    resetControlErrors();       // zero all PID integrators, etc.
    disableWallsControl();      // start clean, no wall?following
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
    float    yaw        = 0.0f;                      /* current heading (deg)        */
    float    step       = initStepDeg;               /* current half-interval        */
    uint32_t bestDist   = getRobotDistanceUm(SENSOR_CENTER);
    float    bestYawDeg = 0.0f;

    for (uint8_t k = 0; k < maxIter; ++k) {

        /* --- probe on the +step side ------------------------------------ */
        inplaceTurn( step * DEG2RAD, force);
        yaw += step;
        uint32_t dPlus = getRobotDistanceUm(SENSOR_CENTER);

        /* --- probe on the -step side ------------------------------------ */
        inplaceTurn(-2.0f * step * DEG2RAD, force);  /* net move: ?step from start  */
        yaw -= 2.0f * step;
        uint32_t dMinus = getRobotDistanceUm(SENSOR_CENTER);

        /* ---------------------------------------------------------------- */
        /* Decide which side is better and centre the search there          */
        /* ---------------------------------------------------------------- */
        if (dPlus < dMinus) {
            /* +step side is better ? go back there                          */
            inplaceTurn( step * DEG2RAD, force);
            yaw += step;

            if (dPlus < bestDist) { bestDist = dPlus; bestYawDeg = yaw; }
        } else {
            /* We are already sitting on the ?step side                      */
            if (dMinus < bestDist) { bestDist = dMinus; bestYawDeg = yaw; }
        }

        /* Shrink the search window                                         */
        step *= 0.5f;
        if (step < 0.05f)          /* ~0.05 ° -> 1 mrad -> resolution reached */
            break;
    }

    /* -------------------------------------------------------------------- */
    /* Return to the orientation that gave the absolute minimum reading     */
    /* -------------------------------------------------------------------- */
    inplaceTurn(-bestYawDeg * DEG2RAD, force);
}

void keepFrontWallDistance(float distance)
{
    int i;
    float diff, frontWallDistance;

    if (!sensorIsWallFront())
        return;

    setMaxForce(getMaxForce() / 2.0f);

    while (true) {
        sideSensorsCloseControl(false);
        sideSensorsFarControl(false);

        squareUpByWiggle(0.5f, 10, getMaxForce());

        frontWallDistance = 0.0f;
        for (i = 0; i < 20; ++i) {
            frontWallDistance += (float) (getRobotDistanceUm(SENSOR_CENTER));
            sleepTicks(2);
        }
        frontWallDistance /= 20;
        diff = frontWallDistance - distance;
        if (fabsf(diff) < KEEP_FRONT_DISTANCE_TOLERANCE_UM)
            break;

        targetStraight(getEncoderAverageDistanceUm(), diff, 0.0f);
    }

    setMaxForce(getMaxForce() * 2.0f);

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
 * Drive one whole cell forward (centre ? centre).
 */
static void forwardOneCell(float cruiseSpeed, float endSpeed)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    setMaxLinearSpeed(cruiseSpeed);

    targetStraight(currentCellStartMicrometers, CELL_DIMENSION_UM, endSpeed);

    enteredNextCell();
}

/**
 * Perform a 90° pivot *in place* while sitting in the cell centre.  No
 * longitudinal motion is required because we already have clearance front and
 * back (1/2?cell each way).
 */
static void pivot90(Movement_t dir, float force)
{
    disableWallsControl();

    if (dir == MOVE_LEFT)
        inplaceTurn( M_PI / 2.0f, force);
    else
        inplaceTurn(-M_PI / 2.0f, force);
}


/**
 * Perform a 180° turn (dead?end recovery) while staying centred.
 */
static void pivot180(float force)
{   
    disableWallsControl();
    
    int16_t dirSign = (int16_t) (rand() % 2) * 2 - 1;      // random left/right pivot

    inplaceTurn(dirSign * M_PI, force);
}

/**
 * @brief  Move forward *n* cells (centre ? centre).
 *
 * The function accelerates once, cruises through the middle cells and decelerates
 * so that it reaches @p endSpeed exactly at the centre of the last cell.
 */
void moveForwardCells(uint8_t nCells, float cruiseSpeed, float endSpeed)
{
    if (nCells == 0) return;

    for (uint8_t k = 1; k < nCells; ++k)
        forwardOneCell(cruiseSpeed, cruiseSpeed);

    forwardOneCell(cruiseSpeed, endSpeed);
}

/**
 * @brief  One?cell step forward.
 */
void moveForwardCenter(float cruiseSpeed, float endSpeed)
{
    forwardOneCell(cruiseSpeed, endSpeed);
}

/**
 * @brief  90° left turn, centre?pivot.
 */
void turnLeftCenter(float force)
{
    pivot90(MOVE_LEFT, force);
}

/**
 * @brief  90° right turn, centre?pivot.
 */
void turnRightCenter(float force)
{
    pivot90(MOVE_RIGHT, force);
}

/**
 * @brief  Dead?end handling: square?up, 180° pivot, step back one cell.
 */
void escapeDeadEnd(float force, float cruiseSpeed, float endSpeed)
{
    if (getRobotDistanceUm(SENSOR_CENTER) < CELL_DIMENSION_UM)
        keepFrontWallDistance(CELL_DIMENSION_UM / 2.0f);

    pivot180(force);
    
    //TODO
    currentCellStartMicrometers = getEncoderAverageDistanceUm() - (int32_t) (CELL_DIMENSION_UM / 2 + SHIFT_AFTER_180_DEG_TURN_UM);
    
    /* After the about?face we just re?use the forward routine. */
    forwardOneCell(cruiseSpeed, endSpeed);
}

/**
 * @brief  Dispatch function similar to the old *move()* but using the new
 *         centre?to?centre primitives.  BACK performs a dead?end escape.
 */
void moveCentered(StepDirection_t dir, float force, float cruiseSpeed, float endSpeed)
{
    switch (dir) {
    case FRONT:  moveForwardCenter(cruiseSpeed, endSpeed);       break;
    case LEFT:   turnLeftCenter(force);     moveForwardCenter(cruiseSpeed, endSpeed);    break;
    case RIGHT:  turnRightCenter(force);    moveForwardCenter(cruiseSpeed, endSpeed);    break;
    case BACK:   escapeDeadEnd(force, cruiseSpeed, endSpeed);    break;
    default:     resetControlErrors();              break;
    }
}
