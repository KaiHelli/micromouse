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
#include "constants.h"


//=============================================================================
//  Wrap helper  ?? works in radians, heading in [?? ? ?)
//=============================================================================
static inline float wrapPi(float angle)
{
    if      (angle >=  M_PI) angle -= 2.0f*M_PI;
    else if (angle <  -M_PI) angle += 2.0f*M_PI;
    
    return angle;
}

//=============================================================================
//  Angle helper  ?? works in radians, heading in (?? ? ?]
//=============================================================================
static float angleError(float current, float target)
{
    float diff = target - current;          // signed error
    while (diff >  M_PI) diff -= 2.0f * M_PI;
    while (diff <= -M_PI) diff += 2.0f * M_PI;
    return diff;                            // (?? ? ?]
}

//=============================================================================
//  TURN (degrees)  ?? spin in place to an absolute heading
//=============================================================================
static volatile bool turnInProgress = false;
static float         turnStartYaw   = 0.0f;
static float         turnTargetYaw  = 0.0f;
static int8_t        turnDirection  = 0;    // +1 CW, ?1 CCW

int16_t turnDegreesCallback(void)
{
    // How many radians left to target? (wrapped to -pi..+pi)
    float error = angleError(mouseAngle[YAW], turnTargetYaw);

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
    turnStartYaw = mouseAngle[YAW];

    // Compute final yaw after turning (wrapped to [0..360))
    float newTarget = turnStartYaw + degrees * DEG2RAD;
    newTarget = wrapPi(newTarget);
    
    // Update global objective
    turnTargetYaw = newTarget;

    // Register callback and start motors
    registerTimerCallback(timer, turnDegreesCallback);

    // Motor power: steer them in correct direction
    steerMotors(100 * turnDirection, powerInPercent);
    setMotorsStandbyState(false);

    while (turnInProgress) { /* busy?wait until callback stops us */ }
}

//=============================================================================
//  MOVE (mm)  ?? drive straight from current XY, keeping yaw fixed
//=============================================================================
static FastPid        movePid;
static volatile bool  moveInProgress     = false;
static float          moveStartX         = 0.0f;
static float          moveStartY         = 0.0f;
static float          moveTargetX        = 0.0f;
static float          moveTargetY        = 0.0f;
static float          moveStartYaw       = 0.0f;
static uint8_t        movePowerPct       = 0;        // 0?100 %

int16_t moveDistanceCallback(void)
{
    float dx = moveTargetX - mousePosition[X];
    float dy = moveTargetY - mousePosition[Y];
    float remaining = sqrtf(dx*dx + dy*dy);          // distance left [mm]

    int16_t step = fastPidStep(&movePid, 0, (int16_t)angleError(mouseAngle[YAW], moveStartYaw));

    int8_t powerL = (int8_t)movePowerPct - (int8_t)step;
    int8_t powerR = (int8_t)movePowerPct + (int8_t)step;

    setMotorPower(MOTOR_LEFT,  powerL);
    setMotorPower(MOTOR_RIGHT, powerR);

    if (remaining <= 0.0f)                           // arrived / overshot
    {
        setMotorsState(MOTORS_BRAKE);
        __delay_ms(100);
        setMotorsStandbyState(true);
        moveInProgress = false;
        return 0;                                   // unregister callback
    }
    return 1;                                       // keep driving
}

void moveDistance(Timer_t timer, int16_t distance, uint8_t powerInPercent, float timer_hz)
{
    if (distance == 0) return;

    moveInProgress = true;
    movePowerPct   = powerInPercent;
    moveStartYaw   = mouseAngle[YAW];

    moveStartX = mousePosition[X];
    moveStartY = mousePosition[Y];

    // Compute absolute target in XY plane from current heading
    float distF = (float)distance;
    moveTargetX = moveStartX + distF * cosf(moveStartYaw);
    moveTargetY = moveStartY + distF * sinf(moveStartYaw);

    // PID keeps the robot aligned with moveStartYaw
    bool status = true;
    fastPidInit(&movePid);
    status &= fastPidConfigure(&movePid, 0.8f, 0.0f, 0.0f, timer_hz, 8, true);
    status &= fastPidSetOutputRange(&movePid, -100, 100);
    if (!status) putsUART1("PID setup failed.\r\n");

    registerTimerCallback(timer, moveDistanceCallback);

    setMotorPower(MOTOR_LEFT,  powerInPercent);
    setMotorPower(MOTOR_RIGHT, powerInPercent);
    setMotorsStandbyState(false);

    while (moveInProgress) { /* busy?wait until callback stops us */ }

    // Snap back to original heading (tiny correction if drifted)
    turnDegrees(timer, -angleError(mouseAngle[YAW], moveStartYaw) * RAD2DEG, powerInPercent);
}

//=============================================================================
//  MOVE (mm)  ?? drive straight from current XY, keeping yaw fixed
//=============================================================================
int16_t moveForward() {
    //resetPid
    //setgoaldistance numCells * celldistance
    //movement state: moving
    //registerTimerCallback(TIMERX, updatePid)
    //update pid will set movement state to NONE and return 0 (unregistering the callback) if goal reached
    //while(movementstae==moving) {} busy wait
    //
    
    uint16_t left = getSensorDistance(SENSOR_LEFT);
    uint16_t right = getSensorDistance(SENSOR_RIGHT);
    uint16_t front = getSensorDistance(SENSOR_CENTER);
    
    int16_t step = fastPidStep(&pid, 0, (int16_t) left-right);
    
    uint8_t defaultPower = 60;
    
    int8_t powerLeft = defaultPower + (int8_t) step;
    int8_t powerRight = defaultPower - (int8_t) step;
    
    setMotorPower(MOTOR_RIGHT, powerRight);
    setMotorPower(MOTOR_LEFT, powerLeft);
    
    //char buffer[100];
    //snprintf(buffer, sizeof(buffer), "Power [%] - Step: %d, Left: %d, Right: %d\r\n", (int8_t) step, powerLeft, powerRight);
    //putsUART1(buffer);
    //snprintf(buffer, sizeof(buffer), "Sensor readings in mm left: %u, right: %u, center: %u\r\n",left, right, front);
    //putsUART1(buffer);
    return 1;
}