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

/** Encoder reference (?m) marking the start of the current maze cell. */
static int32_t currentCellStartMicrometers;

/** Runtime angular acceleration (rad?/?s²). */
static float angularAcceleration;

/* -------------------------------------------------------------------------- */
/* Helper functions                                                           */
/* -------------------------------------------------------------------------- */

void sleepTicks(uint16_t ticks) {
    //TODO
    __delay_ms(ticks * 10);
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
        frOffset = (int32_t) front - (int32_t) MIDDLE_MAZE_DISTANCE_MM;
        uprintf("LR Offset: %ld | FR Offset: %ld\r\n", lrOffset, frOffset);
        uprintf("L: %u F: %u R: %u\r\n", left, front, right);
        
        __delay_ms(500);
    } while (labs(lrOffset) > SENSOR_OFFSET_THRESHOLD || labs(frOffset) > SENSOR_OFFSET_THRESHOLD);
    
    uprintf("Centering done. Don't move anymore.\r\n");
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

    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm();
    
    if (sensorIsWallFront()) {
        frontWallCorrection = (int32_t) getRobotDistanceUm(SENSOR_CENTER) - (int32_t) (CELL_DIMENSION_UM);
        currentCellStartMicrometers += frontWallCorrection;
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void setStartingPosition(void)
{
    currentCellStartMicrometers = (int32_t) getEncoderAverageDistanceUm();
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
    float t, transition, arc, transitionAngle, maxAngularVelocity, angularVelocity, factor;
    int signDir = sign(radians);

    radians = fabsf(radians);

    /* Compute trapezoidal angular profile. */
    angularAcceleration = force * MOUSE_WHEEL_SEPARATION_MM * MILLIMETERS_PER_METER / MOUSE_MOMENT_OF_INERTIA_KGM2;
    maxAngularVelocity = sqrtf(radians / 2.0f * angularAcceleration);
    if (maxAngularVelocity > MOUSE_MAX_ANGULAR_VELOCITY_RADPS)
        maxAngularVelocity = MOUSE_MAX_ANGULAR_VELOCITY_RADPS;

    transition = maxAngularVelocity / angularAcceleration;      // time to accel/decel
    transitionAngle = 0.5f * angularAcceleration * transition * transition;
    arc = radians - 2.0f * transitionAngle;                     // constant-speed arc
    maxAngularVelocity *= signDir;                              // apply direction

    setTargetLinearSpeed(getIdealLinearSpeed());
    disableWallsControl();

    startTime = getTimeInUs();
    while (true) {
        curTime = getTimeInUs();
        t = (float) (curTime - startTime) / MICROSECONDS_PER_SECOND;
        if (t >= 2 * transition + arc / fabsf(maxAngularVelocity))
            break;

        angularVelocity = maxAngularVelocity;
        if (t < transition) {
            factor = t / transition;
            angularVelocity *= sinf(factor * M_PI / 2.0f);
        } else if (t > transition + arc / fabsf(maxAngularVelocity)) {
            factor = (t - (transition + arc / fabsf(maxAngularVelocity))) / transition;
            angularVelocity *= sinf(factor * M_PI / 2.0f);
        }
        setIdealAngularSpeed(angularVelocity);
    }
    setIdealAngularSpeed(0.0f);
}

/* -------------------------------------------------------------------------- */
/* Composite high-level moves                                                 */
/* -------------------------------------------------------------------------- */

void turnBack(float force)
{
    int dirSign;

    if (getRobotDistanceUm(SENSOR_CENTER) < CELL_DIMENSION_UM)
        keepFrontWallDistance(CELL_DIMENSION_UM / 2.0f);

    disableWallsControl();

    dirSign = (int)(rand() % 2) * 2 - 1;      // random left/right pivot
    inplaceTurn(dirSign * M_PI, force);

    currentCellStartMicrometers =
        getEncoderAverageDistanceUm() -
        (CELL_DIMENSION_UM / 2.0f + SHIFT_AFTER_180_DEG_TURN_UM);
}

void turnToStartPosition(float force)
{
    float delta;

    setMaxForce(getMaxForce() / 4.0f);

    turnBack(force);
    delta = MOUSE_START_SHIFT_MM * MICROMETERS_PER_MILLIMETER - currentCellShift();
    targetStraight(getEncoderAverageDistanceUm(), delta, 0.0f);

    setMaxForce(getMaxForce() * 4.0f);

    disableWallsControl();
    resetControlAll();
    enableMouseControl();
    setMotorsState(MOTORS_BRAKE);
}

void moveFront(void)
{
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);

    targetStraight(currentCellStartMicrometers, CELL_DIMENSION_UM, getMaxLinearSpeed());
    enteredNextCell();
}

void parametricMoveFront(float distance, float endLinearSpeed)
{
    targetStraight(getEncoderAverageDistanceUm(), distance, endLinearSpeed);
}

/**
 * @brief Move into the next cell with a 90° in-place turn if requested.
 */
void moveSide(Movement_t turn, float force)
{
    float beforeTurn = CELL_DIMENSION_UM / 2.0f;   // forward half-cell, pivot, forward half-cell

    /* Forward to mid-cell. */
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);
    targetStraight(currentCellStartMicrometers, beforeTurn, 0.0f);

    /* Pivot */
    disableWallsControl();
    if (turn == MOVE_LEFT)
        inplaceTurn(M_PI / 2.0f, force);
    else
        inplaceTurn(-M_PI / 2.0f, force);

    /* Second half cell. */
    sideSensorsCloseControl(true);
    sideSensorsFarControl(false);
    targetStraight(getEncoderAverageDistanceUm(), beforeTurn, getMaxLinearSpeed());
    enteredNextCell();
}

void moveBack(float force)
{
    stopMiddle();
    turnBack(force);
    moveFront();
}

void move(StepDirection_t dir, float force)
{
    switch (dir) {
    case LEFT:
        moveSide(MOVE_LEFT, force);
        break;
    case RIGHT:
        moveSide(MOVE_RIGHT, force);
        break;
    case FRONT:
        moveFront();
        break;
    case BACK:
        moveBack(force);
        break;
    default:
        stopMiddle();
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* Simple sequence executor - handles only front / left / right / stop.       */
/* -------------------------------------------------------------------------- */

void executeMovementSequence(char *seq, float force, PathLanguage_t lang)
{
    int i = 0;
    char cmd;

    (void)lang; // Diagonals not supported in minimal build.

    while (true) {
        cmd = seq[i++];
        switch (cmd) {
        case MOVE_FRONT:
            moveFront();
            break;
        case MOVE_LEFT:
            moveSide(MOVE_LEFT, force);
            break;
        case MOVE_RIGHT:
            moveSide(MOVE_RIGHT, force);
            break;
        case MOVE_STOP:
            stopMiddle();
            turnToStartPosition(force);
            break;
        case MOVE_END:
            return;
        default:
            return;
        }

        //if (collision_detected()) {
        //    return;
        //}
    }
}
