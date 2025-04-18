#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "motors.h"
#include "IOConfig.h"
#include "odometry.h"
#include "timers.h"
#include "fastPID.h"
#include "uart.h"

static volatile bool turnInProgress = false;
static float startYaw = 0.0f;
static float targetYaw = 0.0f;
static int8_t turnDirection = 0;

static float angleError(float current, float target)
{
    // Returns a signed difference from current to target in (-180..180]
    float diff = target - current;
    while (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    while (diff <= -180.0f)
    {
        diff += 360.0f;
    }
    return diff;
}

int16_t turnDegreesCallback(void)
{
    // How many degrees left to target? (wrapped to -180..+180)
    float error = angleError(mouseAngle[YAW], targetYaw);

    // If turning CW (turnDirection > 0), we're done once error <= 0
    // If turning CCW (turnDirection < 0), we're done once error >= 0
    if ((turnDirection > 0 && error <= 0) ||
        (turnDirection < 0 && error >= 0))
    {
        setMotorsState(MOTORS_BRAKE);
        __delay_ms(100);
        setMotorsStandbyState(true);
        turnInProgress = false;
        return 0; // Unregister callback
    }

    return 1; // Keep going
}

void turnDegrees(Timer_t timer, int16_t degrees, uint8_t powerInPercent)
{
    // Limit the input to ±359
    degrees = degrees % 360;
    if (degrees == 0)
    {
        return; // No turn needed
    }

    // Decide direction (+1 CW, -1 CCW)
    turnDirection  = (degrees > 0) ? 1 : -1;
    turnInProgress = true;

    // Record current yaw as start
    startYaw = mouseAngle[YAW];

    // Compute final yaw after turning (wrapped to [0..360))
    float newTarget = startYaw + degrees;
    while (newTarget >= 360.0f)
    {
        newTarget -= 360.0f;
    }
    while (newTarget < 0.0f)
    {
        newTarget += 360.0f;
    }
    targetYaw = newTarget;

    // Register callback and start motors
    registerTimerCallback(timer, turnDegreesCallback);

    // Motor power: steer them in correct direction
    steerMotors(100 * turnDirection, powerInPercent);
    setMotorsStandbyState(false);

    // Wait until we finish the turn
    while (turnInProgress)
    {
        // This loop blocks until turnDegreesCallback() stops the turn
    }
}

static FastPid pid;
static volatile bool moveInProgress = false;
static float startPos = 0.0f;
static float targetPos = 0.0f;
static float moveStartYaw = 0.0f;
static int8_t moveDirection = 0;
static uint8_t movePowerInPercent = 0;

int16_t moveDistanceCallback(void)
{
    // How many millimeters left to target?
    float error = mousePosition[Y] - targetPos;
    
    int16_t step = fastPidStep(&pid, 0, (int16_t) angleError(mouseAngle[YAW], moveStartYaw));
    
    uint8_t defaultPower = movePowerInPercent;
    
    int8_t powerLeft = defaultPower - (int8_t) step;
    int8_t powerRight = defaultPower + (int8_t) step;
    
    setMotorPower(MOTOR_RIGHT, powerRight);
    setMotorPower(MOTOR_LEFT, powerLeft);

    // If moving FW (moveDirection > 0), we're done once error <= 0
    // If moving BW (moveDirection < 0), we're done once error >= 0
    if ((moveDirection > 0 && error >= 0) ||
        (moveDirection < 0 && error <= 0))
    {
        setMotorsState(MOTORS_BRAKE);
        __delay_ms(100);
        setMotorsStandbyState(true);
        moveInProgress = false;
        return 0; // Unregister callback
    }

    return 1; // Keep going
}


//TODO: Negative distances don't work
void moveDistance(Timer_t timer, int16_t distance, uint8_t powerInPercent)
{
    // Decide direction (+1 FW, -1 BW)
    moveDirection  = (distance > 0) ? 1 : -1;
    moveInProgress = true;

    // Record current yaw, position and power as start
    startPos = mousePosition[Y];
    moveStartYaw = mouseAngle[YAW];
    movePowerInPercent = powerInPercent;

    // Compute final position after moving
    targetPos = startPos + (float) distance;
    
    // Initialize PID to drive straight
    bool status = true;
    
    fastPidInit(&pid);
    status &= fastPidConfigure(&pid, 0.8f, 0.0f, 0.0f, 500.0f, 8, true);
    status &= fastPidSetOutputRange(&pid, -100, 100);
    
    if (!status) {
        putsUART1("PID setup failed.\r\n");
    }
    
    // Register callback and start motors
    registerTimerCallback(timer, moveDistanceCallback);

    // Motor power: steer them in correct direction
    setMotorPower(MOTOR_LEFT, (int8_t) powerInPercent * moveDirection);
    setMotorPower(MOTOR_RIGHT, (int8_t) powerInPercent * moveDirection);
    setMotorsStandbyState(false);

    // Wait until we finish the move
    while (moveInProgress)
    {
        // This loop blocks until moveDistanceCallback() stops the movement
    }
    
    // Fix final angle error
    turnDegrees(timer, (int16_t) -angleError(mouseAngle[YAW], moveStartYaw), powerInPercent);
}