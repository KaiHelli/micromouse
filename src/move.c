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
#include "sensors.h"
#include "motorEncoders.h"

#define WALL_SEEN_THRESHOLD_MM     75          // side wall "in range"?
#define WALL_TARGET_OFFSET_MM      30          // keep this gap to one wall
#define BRAKE_DISTANCE_MM          20          // start decelerating here
#define MIN_DRIVE_POWER_PCT        25          // don't stall below this
#define FRONT_STOP_MM              10          // hard stop before collision
#define RAMP_ANGLE_DEG             45          // turn: begin braking here
#define MIN_TURN_POWER_PCT         20
#define TURN_EXIT_EPSILON          1

#define TURN_PID_KP      0.75f       // tune for your bot
#define TURN_PID_KD      0.0f
#define TURN_PID_KI      0.0f      // small I to overcome static friction
#define TURN_PID_KF      0.8f
#define TURN_MAX_ACC_DPS2   3900.0f    // deg/s²  (wheel torque / inertia) (measured upper limit: np.array([0,88,565,1290,2095,2745,3272,3734,4067,4327,4526,4676,4801,4886,5004]) / 10)
#define TURN_MAX_VEL_DPS    501.4f    // deg/s (measured upper limit)
#define TURN_SETTLE_EPSILON    0.5f      // ° error small enough
#define TURN_SETTLE_OUTPUT     5         // |PID| small enough to call it "settled"


//=============================================================================
// Make IMU yaw continuous: [-pi,pi)  ->  ... -2pi, 0, +2pi, +4pi ... 
//=============================================================================
static float  yawOff   = 0.0f;      /* accumulated +/-2pi multiples   */
static float  yawLast  = 0.0f;      /* previous wrapped reading    */

static inline float unwrapYaw(float yawWrapped /* in [-pi,pi) */)
{
    float d = yawWrapped - yawLast;

    if      (d >  M_PI) yawOff -= 2.0f * M_PI;   /* +pi to -pi crossing  */
    else if (d < -M_PI) yawOff += 2.0f * M_PI;   /* -pi to +pi crossing  */

    yawLast = yawWrapped;
    return yawWrapped + yawOff;                  /* continuous yaw    */
}

//=============================================================================
//  Wrap helper angle works in radians, heading in [-pi, pi)
//=============================================================================
static inline float wrapPi(float angle)
{
    if      (angle >=  M_PI) angle -= 2.0f*M_PI;
    else if (angle <  -M_PI) angle += 2.0f*M_PI;
    
    return angle;
}

//=============================================================================
//  Angle helper works in radians, heading in [-pi, pi)
//=============================================================================
static float angleError(float current, float target)
{
    float diff = target - current;          // signed error
    
    return wrapPi(diff);
}

// ============================================================================
//  TURN (adaptive)
// ============================================================================
static FastPid         turnPid;                       // outer w-controller
static volatile bool   turnInProgress   = false;
static float           turnTargetYaw    = 0.0f;
static int8_t          turnDirection    = 0;          // +1 = CW, -1 = CCW
static float           turnCruiseVelDps = 0.0f;       // cruise w [deg/s]
static float           velCmd           = 0.0f;       // commanded w       [deg/s]
static float           velMeas          = 0.0f;       // measured  w       [deg/s]
static float           turnDt           = 0.0f;       // callback period [s] (set in init routine)
static uint16_t        settleCtr        = 0;
static bool            turnInit         = true;

static inline float signf(float x){ return (x>0) - (x<0); }

static int16_t turnDegreesCallback(void)
{
    static float lastAngleDeg = 0.0f;

    /* ---- current state --------------------------------------------- */
    float yawNow  = unwrapYaw(mouseAngle[YAW]);           // continuous yaw
    
    /* ---------------- state & error ----------------------------------- */
    float angNowDeg = yawNow * RAD2DEG;
    float errDeg  = (turnTargetYaw - yawNow) * RAD2DEG;   // signed error
    float dt        = turnDt;
    velMeas         = (angNowDeg - lastAngleDeg) / dt;
    
    if (turnInit) {
        turnInit = false;
        return;
    }

    /* ---------------- trapezoidal feed-forward ------------------------ */
    float velLim = sqrtf(2.0f * TURN_MAX_ACC_DPS2 * fabsf(errDeg));  // v = sqrt(2·a·delta_theta)
    if (velLim > turnCruiseVelDps) velLim = turnCruiseVelDps;
    velLim *= signf(errDeg);

    /* acceleration clamp */
    float dvMax = TURN_MAX_ACC_DPS2 * dt;
    float dv    = velLim - velCmd;
    if (fabsf(dv) > dvMax) dv = signf(dv) * dvMax;
    velCmd += dv;                                       /* new w setpoint */
    
    /* ---------------- outer w-PID ------------------------------------- */
    int16_t trimCdps = fastPidStep(&turnPid,
                                   (int32_t)(velCmd  * 10.0f),     // setpoint  [ddeg/s]
                                   (int32_t)(velMeas * 10.0f));    // actual    [ddeg/s]
    float   omegaDps = velCmd + (float)trimCdps / 10.0f;           // corrected w
    

    /* ---------------- w -> wheel linear speed [mm/s] ------------------- */
    float omegaRad   = omegaDps * DEG2RAD;
    float v_mmps     = omegaRad * (WHEEL_BASE_MM * 0.5f);           // v = w·r

    setMotorSpeedLeft ( +v_mmps );   /* left wheel forward  */
    setMotorSpeedRight( -v_mmps );   /* right wheel reverse */
    
    static uint16_t i = 0;
    if (i % 1 == 0) {
        char buf[100];
        //snprintf(buf, sizeof(buf), "velMeas: %ld, velCmd: %ld, trimCdps: %d, omegaDps: %.2f, errDeg: %.2f, vLeft: %.2f\r\n", (int32_t)(velMeas * 10.0f), (int32_t)(velCmd  * 10.0f), trimCdps, omegaDps, errDeg, v_mmps);
        //snprintf(buf, sizeof(buf), "velMeas: %ld\r\n", (int32_t)(velMeas * 10.0f));
        snprintf(buf, sizeof(buf), "an: %.6f la: %.6f, vm: %ld\r\n", angNowDeg, lastAngleDeg, (int32_t)(velMeas * 10.0f));
        putsUART1Str(buf);
    }
    i++;
    lastAngleDeg    = angNowDeg;

    /* ---------------- settle detection -------------------------------- */
    if (fabsf(errDeg) < TURN_SETTLE_EPSILON &&
        fabsf(velMeas) < TURN_SETTLE_OUTPUT)
    {
        if (++settleCtr >= 30) {                 /* 30 × 1 kHz -> 30 ms */
            setMotorSpeedLeft (0);
            setMotorSpeedRight(0);
            turnInProgress = false;
            
            setMotorsStandbyState(true);
            
            return 0;
        }
    } else {
        settleCtr = 0;
    }
    return 1;
}

void turnDegrees(Timer_t timer, int16_t degrees, float cruiseDegPerSec, float timer_hz)
{
    if (degrees == 0) return;
    
    /* ----  set up the unwrapper  ------------------------------------ */
    yawLast = mouseAngle[YAW];      /* current wrapped IMU reading      */
    yawOff  = 0.0f;                 /* no offset yet                    */
    
    turnInit          = true;
    turnDirection     = (degrees > 0) ? 1 : -1;
    turnCruiseVelDps  = fabsf(cruiseDegPerSec);
    turnInProgress    = true;
    velCmd            = 0.0f;
    settleCtr         = 0;
    turnDt            = 1.0f / timer_hz;

    turnTargetYaw = mouseAngle[YAW] + degrees * DEG2RAD;

    /* --------------- init outer PID (w loop) -------------------------- */
    bool status;
    fastPidInit(&turnPid);
    fastPidConfigure (&turnPid, TURN_PID_KP, TURN_PID_KI, TURN_PID_KD, TURN_PID_KF, timer_hz, 8, true);
    fastPidSetOutputRange(&turnPid, -(int16_t)(turnCruiseVelDps * 10.0f), +(int16_t)(turnCruiseVelDps * 10.0f));

    if (fastPidHasConfigError(&turnPid)) {
        putsUART1Str("Failed to setup PID for turning.\r\n");
        return;
    }

    /* --------------- wake the wheel-speed PIDs ------------------------ */
    setMotorsStandbyState(false);

    /* small kick to overcome static friction */
    float kickOmega   = turnDirection * fminf(turnCruiseVelDps, 30.0f);      // 30 °/s cap
    float kickV_mmps  = kickOmega * DEG2RAD * (WHEEL_BASE_MM * 0.5f);
    setMotorSpeedLeft ( +kickV_mmps );
    setMotorSpeedRight( -kickV_mmps );

    registerTimerCallback(timer, turnDegreesCallback);

    while (turnInProgress) { /* busy-wait */}
}

// ============================================================================
//  MOVE (3 wall modes + adaptive braking)
// ============================================================================
typedef enum { WALL_NONE=0, WALL_SINGLE_LEFT, WALL_SINGLE_RIGHT, WALL_BOTH } WallMode;

static FastPid        movePid;
static volatile bool  moveInProgress      = false;
static float          moveTargetX         = 0.0f;
static float          moveTargetY         = 0.0f;
static float          moveStartYaw        = 0.0f;
static uint8_t        moveCruisePowerPct  = 0;
static WallMode       moveLastMode        = WALL_NONE;   

static inline WallMode detectWallMode(uint16_t left, uint16_t right)
{
    bool l = (left  < WALL_SEEN_THRESHOLD_MM);
    bool r = (right < WALL_SEEN_THRESHOLD_MM);
    if (l && r)            return WALL_BOTH;
    if (l && !r)           return WALL_SINGLE_LEFT;
    if (!l && r)           return WALL_SINGLE_RIGHT;
    return WALL_NONE;
}

static int16_t moveDistanceCallback(void)
{
    float dx = moveTargetX - mousePosition[X];
    float dy = moveTargetY - mousePosition[Y];
    float remaining = sqrtf(dx*dx + dy*dy);          // mm to goal

    // --------------------------------------------------------------------
    //  1. read sensors + choose control mode
    // --------------------------------------------------------------------
    uint16_t left  = getSensorDistance(SENSOR_LEFT);
    uint16_t right = getSensorDistance(SENSOR_RIGHT);
    WallMode mode  = detectWallMode(left, right);

    if (mode != moveLastMode) {
        fastPidClear(&movePid);             // dump I-sum & lastError
        moveLastMode = mode;
    }
    
    int16_t controlError = 0;

    switch (mode)
    {
        case WALL_BOTH:
            // stay equal distance between two walls
            controlError = (int16_t)(left - right);
            break;

        case WALL_SINGLE_LEFT:
            controlError = (int16_t)(WALL_TARGET_OFFSET_MM - left);
            break;

        case WALL_SINGLE_RIGHT:
            controlError = (int16_t)(right - WALL_TARGET_OFFSET_MM);
            break;

        case WALL_NONE:
        default:
            controlError = (int16_t)(angleError(mouseAngle[YAW], moveStartYaw) * 1000); // convert rad->milli-rad for PID resolution
            break;
    }

    int16_t pidStep = fastPidStep(&movePid, 0, controlError);

    // --------------------------------------------------------------------
    //  2. adaptive forward power (brake before goal or wall)
    // --------------------------------------------------------------------
    uint16_t front = getSensorDistance(SENSOR_CENTER);
    float    brakeDist = remaining;

    if (front < FRONT_STOP_MM) brakeDist = 0;                // emergency
    else if (front < BRAKE_DISTANCE_MM)                      // brake for front wall too
        brakeDist = fminf(brakeDist, (float)(front - FRONT_STOP_MM));

    uint8_t dynPower = moveCruisePowerPct;
    if (brakeDist < BRAKE_DISTANCE_MM) {
        dynPower = MIN_DRIVE_POWER_PCT +
            (uint8_t)((moveCruisePowerPct - MIN_DRIVE_POWER_PCT) *
                      brakeDist / BRAKE_DISTANCE_MM);
        if (dynPower < MIN_DRIVE_POWER_PCT) dynPower = MIN_DRIVE_POWER_PCT;
    }

    int8_t powerL = (int8_t)dynPower - (int8_t)pidStep;
    int8_t powerR = (int8_t)dynPower + (int8_t)pidStep;

    setMotorPower(MOTOR_LEFT,  powerL);
    setMotorPower(MOTOR_RIGHT, powerR);

    // --------------------------------------------------------------------
    //  3. stop criteria
    // --------------------------------------------------------------------
    bool goalReached = (remaining <= 0.0f);
    bool frontTooClose = (front <= FRONT_STOP_MM);

    if (goalReached || frontTooClose)
    {
        setMotorsState(MOTORS_BRAKE);
        __delay_ms(80);
        setMotorsStandbyState(true);
        moveInProgress = false;
        return 0;
    }
    return 1;
}

void moveDistance(Timer_t timer,
                  int16_t distance,
                  uint8_t powerInPercent,
                  float   timer_hz)
{
    if (distance == 0) return;

    moveInProgress      = true;
    moveCruisePowerPct  = powerInPercent;
    moveStartYaw        = mouseAngle[YAW];

    float distF = (float)distance;
    moveTargetX = mousePosition[X] + distF * cosf(moveStartYaw);
    moveTargetY = mousePosition[Y] + distF * sinf(moveStartYaw);

    fastPidInit(&movePid);
    fastPidConfigure (&movePid, 1.0f, 0.0f, 0.0f, 0.0f, timer_hz, 8, true);
    fastPidSetOutputRange(&movePid, -100, 100);
    
    if (fastPidHasConfigError(&turnPid)) {
        putsUART1Str("Failed to setup PID for moving.\r\n");
        return;
    }
    
    registerTimerCallback(timer, moveDistanceCallback);

    setMotorPower(MOTOR_LEFT,  powerInPercent);
    setMotorPower(MOTOR_RIGHT, powerInPercent);
    setMotorsStandbyState(false);

    while (moveInProgress) { /* busy-wait */ }

    // tiny correction back to start yaw, if we ended "no-wall"
    turnDegrees(timer,
                -angleError(mouseAngle[YAW], moveStartYaw) * RAD2DEG,
                powerInPercent, timer_hz    );
}
