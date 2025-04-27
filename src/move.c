#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "IOConfig.h"
#include "odometry.h"
#include "timers.h"
#include "fastPID.h"
#include "uart.h"
#include "constants.h"
#include "sensors.h"
#include "motors.h"
#include "motorEncoders.h"

#define WALL_SEEN_THRESHOLD_MM_ON      70      // switch out of sight to in sight at this threshold
#define WALL_SEEN_THRESHOLD_MM_OFF     80      // switch in sight to out of sight at this threshold
#define WALL_TARGET_OFFSET_MM      50          // keep this gap to one wall
#define FRONT_STOP_MM              10          // hard stop before collision

#define TURN_PID_KP             0.08f       // tune for your bot
#define TURN_PID_KD             0.0f
#define TURN_PID_KI             0.0f        // small I to overcome static friction
#define TURN_PID_KF             0.0f
#define TURN_ACC_SAFETY_MARGIN  0.1f
#define TURN_MAX_ACC_DPS2       TURN_ACC_SAFETY_MARGIN * 3000.0f    // deg/s²  (wheel torque / inertia)
#define TURN_MAX_VEL_DPS        700.0f    // deg/s (measured upper limit)
#define TURN_SETTLE_EPSILON     0.5f   // ° error small enough
#define TURN_SETTLE_OUTPUT      5      // |PID| small enough to call it "settled"


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
    
    /*
    static uint16_t i = 0;
    if (i % 1 == 0) {
        uint8_t buffer[10];
        size_t idx = 0;

        buffer[idx++] = FRAME_START_BYTE;

        memcpy(&buffer[idx], &velMeas, sizeof(velMeas));
        idx += sizeof(velMeas);

        buffer[idx++] = FRAME_END_BYTE;

        putsUART1(buffer, idx);
        
        //char buf[100];
        //snprintf(buf, sizeof(buf), "an: %.6f la: %.6f, vm: %ld\r\n", angNowDeg, lastAngleDeg, (int32_t)(velMeas * 10.0f));
        //putsUART1Str(buf);
    }
    i++;
    */
    
    lastAngleDeg    = angNowDeg;

    if (turnInit) {
        turnInit = false;
        return 1;
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
    
    /*
    static uint16_t i = 0;
    if (i % 3 == 0) {
        char buf[100];
        snprintf(buf, sizeof(buf), "vm %ld, vc %ld, trimCdps %d, omegaDps %.2f\r\n", (int32_t)(velMeas * 10.0f), (int32_t)(velCmd  * 10.0f), trimCdps, omegaDps * 10);
        //snprintf(buf, sizeof(buf), "velMeas: %ld, velCmd: %ld, trimCdps: %d, omegaDps: %.2f, errDeg: %.2f, vLeft: %.2f\r\n", (int32_t)(velMeas * 10.0f), (int32_t)(velCmd  * 10.0f), trimCdps, omegaDps, errDeg, v_mmps);
        putsUART1Str(buf);
    }
    i++;
    */

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
    
    if (cruiseDegPerSec > TURN_MAX_VEL_DPS) {
        cruiseDegPerSec = TURN_MAX_VEL_DPS;
    }
    
    turnInit          = true;
    turnDirection     = (degrees > 0) ? 1 : -1;
    turnCruiseVelDps  = fabsf(cruiseDegPerSec);
    turnInProgress    = true;
    velCmd            = 0.0f;
    settleCtr         = 0;
    turnDt            = 1.0f / timer_hz;

    turnTargetYaw = mouseAngle[YAW] + degrees * DEG2RAD;

    /* --------------- init outer PID (w loop) -------------------------- */
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
#define MOVE_PID_KP               0.40f
#define MOVE_PID_KI               0.00f
#define MOVE_PID_KD               0.00f
#define MOVE_PID_KF               0.00f

#define MOVE_ACC_SAFETY_MARGIN  0.8f
#define MOVE_MAX_ACC_MMPS2      MOVE_ACC_SAFETY_MARGIN * WHEEL_MAX_ACC * (M_PI / 180.0f) * WHEEL_RADIUS_MM      // mm / s²  (motor torque ÷ mass)
#define MOVE_MAX_VEL_MMPS       WHEEL_MAX_VEL * (M_PI / 180.0f) * WHEEL_RADIUS_MM                               // mm / s   (tested top speed)

#define MOVE_SETTLE_DISTANCE_MM    1.0f      // "close enough" for goal test

typedef enum { WALL_NONE=0, WALL_SINGLE_LEFT, WALL_SINGLE_RIGHT, WALL_BOTH } WallMode;

static FastPid        movePid;
static volatile bool  moveInProgress      = false;
static float          moveDist            = 0.0f;
static float          moveStartX          = 0.0f;
static float          moveStartY          = 0.0f;
static float          moveLastYaw         = 0.0f;
static WallMode       moveLastMode        = WALL_NONE;
static float          moveCruiseVel_mmps  = 0.0f; 
static float          moveTimerDt         = 0.0f;
static int8_t         moveDirection       = 1;      // +1 = forward, ?1 = reverse

static bool wallSeenHys(uint16_t d, bool prev)
{
    return prev ? (d < WALL_SEEN_THRESHOLD_MM_OFF)       // already ?seen? ? stay until completely gone
                : (d < WALL_SEEN_THRESHOLD_MM_ON);       // not yet seen ? switch only when really close
}

static WallMode detectWallMode(uint16_t left, uint16_t right)
{
    static bool lSeen = false, rSeen = false;

    lSeen = wallSeenHys(left,  lSeen);
    rSeen = wallSeenHys(right, rSeen);

    if (lSeen && rSeen)  return WALL_BOTH;
    if (lSeen)           return WALL_SINGLE_LEFT;
    if (rSeen)           return WALL_SINGLE_RIGHT;
    return WALL_NONE;
}

static inline float dot2(float ax, float ay, float bx, float by)
{
    return ax * bx + ay * by;
}

static int16_t moveDistanceCallback(void)
{
    float dx = moveStartX - mousePosition[X];
    float dy = moveStartY - mousePosition[Y];

    /* projection of the remaining?vector onto the commanded axis   */
    float remainingSigned   = moveDist - sqrtf(dot2(dx, dy, dx, dy));
    float remaining         = fabs(remainingSigned);
    
    // --------------------------------------------------------------------
    //  0.   trapezoidal feed-forward  (linear motion)
    // --------------------------------------------------------------------
    static float velCmd_mmps = 0.0f;   // persistent between calls

    /* symmetric (forward/back) trapezoid --------------------------- */
    float velLim = sqrtf(2.0f * MOVE_MAX_ACC_MMPS2 * remaining);
    if (velLim > moveCruiseVel_mmps) velLim = moveCruiseVel_mmps;
    velLim *= signf(remainingSigned) * moveDirection;    // final sign (fwd / rev)

    /* acceleration clamp */
    float dvMax = MOVE_MAX_ACC_MMPS2 * moveTimerDt;
    float dv    = velLim - velCmd_mmps;
    if (fabsf(dv) > dvMax) dv = signf(dv) * dvMax;
    velCmd_mmps += dv;
    
    // --------------------------------------------------------------------
    //  1. read sensors + choose control mode
    // --------------------------------------------------------------------
    uint16_t left  = getSensorDistance(SENSOR_LEFT);
    uint16_t right = getSensorDistance(SENSOR_RIGHT);
    uint16_t front = getSensorDistance(SENSOR_CENTER);
    
    WallMode mode  = detectWallMode(left, right);

    if (mode != moveLastMode) {
        fastPidClear(&movePid);             // dump I-sum & lastError
        moveLastMode = mode;
    }
    
    if (mode != WALL_NONE) {
        moveLastYaw = mouseAngle[YAW];
    }
    
    int16_t controlError = 0;

    switch (mode)
    {
        case WALL_BOTH:
            // stay equal distance between two walls
            controlError = (int16_t)(right - left);
            break;

        case WALL_SINGLE_LEFT:
            controlError = (int16_t)(WALL_TARGET_OFFSET_MM - left);
            break;

        case WALL_SINGLE_RIGHT:
            controlError = (int16_t)(right - WALL_TARGET_OFFSET_MM);
            break;

        case WALL_NONE:
        default:
            controlError = (int16_t)(angleError(mouseAngle[YAW], moveLastYaw) * 1000); // convert rad->milli-rad for PID resolution
            break;
    }

    int16_t pidStep = fastPidStep(&movePid, 0, controlError);
    
    /*
    uint8_t i = 0;
    if (i % 1 == 0) {
        char buf[70];
        snprintf(buf, sizeof(buf), "%d ", (int)mode);
        putsUART1Str(buf);
        
        i = 0;
    }
    i++;
    */
    
    // --------------------------------------------------------------------
    //  2.   steering correction   (unchanged, but convert ?%-of-max? to mm/s)
    // --------------------------------------------------------------------
    /* pidStep is still -100 -> +100   -> convert to mm/s */
    float vCorr_mmps = (float)pidStep * 0.01f * moveCruiseVel_mmps;

    float vLeft_mmps  =  velCmd_mmps - vCorr_mmps;
    float vRight_mmps =  velCmd_mmps + vCorr_mmps;

    setMotorSpeedLeft ( vLeft_mmps  );
    setMotorSpeedRight( vRight_mmps );

    // --------------------------------------------------------------------
    //  3. stop criteria
    // --------------------------------------------------------------------
    bool goalReached    = (remaining <= MOVE_SETTLE_DISTANCE_MM);
    bool frontTooClose  = (front     <= FRONT_STOP_MM);

    if (goalReached || frontTooClose)
    {
        setMotorSpeedLeft (0);
        setMotorSpeedRight(0);
        moveInProgress = false;
        setMotorsStandbyState(true);
        return 0;
    }
    return 1;
}

void moveDistance(Timer_t timer, int16_t distance, float cruise_mmps, float timer_hz)
{
    if (distance == 0) return;

    /* signed direction ------------------------------------------------ */
    moveDirection      = (distance > 0) ? 1 : -1;        // save for callback

    if (cruise_mmps > MOVE_MAX_VEL_MMPS) {
        cruise_mmps = MOVE_MAX_VEL_MMPS;
    }

    moveInProgress     = true;
    moveCruiseVel_mmps = fabsf(cruise_mmps);             // keep profile maths positive
    moveTimerDt        = 1.0f / timer_hz;

    moveDist         = (float) distance;
    moveStartX       = mousePosition[X];
    moveStartY       = mousePosition[Y];
    moveLastYaw      = mouseAngle[YAW];

    /* PID set?up unchanged ? */
    fastPidInit(&movePid);
    fastPidConfigure(&movePid, MOVE_PID_KP, MOVE_PID_KI, MOVE_PID_KD, MOVE_PID_KF, timer_hz, 8, true);
    fastPidSetOutputRange(&movePid, -100, 100);

    if (fastPidHasConfigError(&movePid)) {
        putsUART1Str("Failed to setup PID for moving.\r\n");
        return;
    }

    setMotorSpeedLeft (0.0f);
    setMotorSpeedRight(0.0f);
    setMotorsStandbyState(false);

    registerTimerCallback(timer, moveDistanceCallback);

    while (moveInProgress) { /* busy?wait */ }

    // tiny correction back to start yaw, if we ended "no-wall"
    // TODO: Tune speed
    //turnDegrees(timer, -angleError(mouseAngle[YAW], moveStartYaw) * RAD2DEG, 90.0f, timer_hz);
}

#define SMALL_ROTATION_ANGLE 10.0 
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
void calibrateGlobalOrientation(){
    float startYaw = mouseAngle[YAW];
    
    uint16_t leftCenter = getSensorDistance(SENSOR_LEFT);
    uint16_t rightCenter = getSensorDistance(SENSOR_RIGHT);

    turnDegrees(TIMER_1, SMALL_ROTATION_ANGLE, 45.0f, 100.0f);
    __delay_ms(20);
    float yawLeft = mouseAngle[YAW];
    uint16_t leftAtLeft = getSensorDistance(SENSOR_LEFT);
    uint16_t rightAtLeft = getSensorDistance(SENSOR_RIGHT);

    turnDegrees(TIMER_1, -2 * SMALL_ROTATION_ANGLE, 45.0f, 100.0f);
    __delay_ms(20);
    float yawRight = mouseAngle[YAW];
    uint16_t leftAtRight = getSensorDistance(SENSOR_LEFT);
    uint16_t rightAtRight = getSensorDistance(SENSOR_RIGHT);

    turnDegrees(TIMER_1, SMALL_ROTATION_ANGLE, 45.0f, 100.0f);
    __delay_ms(20);

    float deltaYaw = yawRight - yawLeft;
    if (fabsf(deltaYaw) < 1e-6f) deltaYaw = 1e-6f; 

    float deltaLeft = (float)(leftAtRight - leftAtLeft);
    float deltaRight = (float)(rightAtRight - rightAtLeft);

    float averageSlope = (deltaLeft + deltaRight) / (2.0f * deltaYaw);

    float correctionAngle = atanf(averageSlope * DEG_TO_RAD) * RAD_TO_DEG;

    turnDegrees(TIMER_1, -correctionAngle, 100.0f, 45.0f); 
    __delay_ms(20);

    float mazeHeading = wrapPi(mouseAngle[YAW]);
    uprintf("Maze heading orientation: %.2f %.2f \n", mazeHeading, correctionAngle);
}
