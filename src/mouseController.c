#include "mouseController.h"
#include "fastPID.h"
#include "motorEncoders.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include "globalTimers.h"
#include "move.h"
#include "constants.h"
#include "imu.h"
#include "odometry.h"
#include "atomic.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define SIDE_WALL_DETECTION ((uint32_t) (CELL_DIMENSION_UM * 0.90))
// #define FRONT_WALL_DETECTION (CELL_DIMENSION_MM * 1.5)
#define FRONT_WALL_DETECTION ((uint32_t) (CELL_DIMENSION_UM * 0.90))

/*
#define MCTRL_LIN_PID_KP                8.0f
#define MCTRL_LIN_PID_KD                16.0f
#define MCTRL_ANG_PID_KP                0.05f
#define MCTRL_ANG_PID_KD                1.0f
#define MCTRL_ANG_SENS_PID_KP           2.0f
#define MCTRL_ANG_SENS_PID_KI           4.0f
*/

#define MCTRL_LIN_FF_TM                 (0.07839f)          // forward time constant
#define MCTRL_LIN_FF_KM                 (1.0f / 171.752f)   // (m/s) / pwm %
#define MCTRL_LIN_FF_KS                 (2.988f)

#define MCTRL_ANG_FF_TM                 (0.03573f)         // forward time constant
#define MCTRL_ANG_FF_KM                 (1.0f / 8.351f)    // (rad/s) / pwm %
#define MCTRL_ANG_FF_KS                 (4.079f)

#define MCTRL_LIN_FF_ZETA               (0.707f)
#define MCTRL_LIN_FF_TD                 (MCTRL_LIN_FF_TM)

#define MCTRL_ANG_FF_ZETA               (0.707f)
#define MCTRL_ANG_FF_TD                 (MCTRL_ANG_FF_TM)

//#define MCTRL_LIN_PID_KP                (16.0f * MCTRL_LIN_FF_TM / (MCTRL_LIN_FF_KM * MCTRL_LIN_FF_ZETA * MCTRL_LIN_FF_ZETA * MCTRL_LIN_FF_TD * MCTRL_LIN_FF_TD))
//#define MCTRL_LIN_PID_KD                ((8.0f * MCTRL_LIN_FF_TM - MCTRL_LIN_FF_TD) / (MCTRL_LIN_FF_KM * MCTRL_LIN_FF_TD))
//#define MCTRL_ANG_PID_KP                (16.0f * MCTRL_ANG_FF_TM / (MCTRL_ANG_FF_KM * MCTRL_ANG_FF_ZETA * MCTRL_ANG_FF_ZETA * MCTRL_ANG_FF_TD * MCTRL_ANG_FF_TD))
//#define MCTRL_ANG_PID_KD                ((8.0f * MCTRL_ANG_FF_TM - MCTRL_ANG_FF_TD) / (MCTRL_ANG_FF_KM * MCTRL_ANG_FF_TD))
#define MCTRL_LIN_PID_KP                (0.0f)
#define MCTRL_LIN_PID_KD                (0.0f)  //0.040756f
#define MCTRL_ANG_PID_KP                (0.0f)  //0.5f
#define MCTRL_ANG_PID_KD                (0.0f)  //0.05f
#define MCTRL_ANG_SENS_PID_KP           (10.0f)
#define MCTRL_ANG_SENS_PID_KI           (4.0f)

typedef struct {
    float Ks;       // static bias               (PWM%)
    float Kv;       // speed gain                (PWM% * s / m   or   PWM% · s / rad)
    float Ka;       // acceleration gain         (PWM% * s^2 / m  or   PWM% · s^2 / rad)
    float prev_v;   // previous target speed     (m/s or rad/s)
} FFProfile_t;

static FFProfile_t ffLin = {
    .Ks = MCTRL_LIN_FF_KS,
    .Kv = 1.0f / MCTRL_LIN_FF_KM,
    .Ka = MCTRL_LIN_FF_TM / MCTRL_LIN_FF_KM,
    .prev_v = 0.0f
};

static FFProfile_t ffAng = {
    .Ks = MCTRL_ANG_FF_KS,
    .Kv = 1.0f / MCTRL_ANG_FF_KM,
    .Ka = MCTRL_ANG_FF_TM / MCTRL_ANG_FF_KM,
    .prev_v = 0.0f
};

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDGains_t;

static volatile PIDGains_t pidLin;
static volatile PIDGains_t pidAng;
static volatile PIDGains_t pidAngSens;

static volatile float maxForce = MOUSE_WHEEL_MAX_FORCE_CONT_N;
static volatile float maxLinearSpeed = MOUSE_WHEEL_MAX_VEL_MMPS / MILLIMETERS_PER_METER;

static volatile float pidFrequency = 0.0f;

static volatile float targetLinearSpeed = 0.0f;
static volatile float idealLinearSpeed = 0.0f;
static volatile float idealAngularSpeed = 0.0f;

static volatile float linearError = 0.0f;
static volatile float angularError = 0.0f;
static volatile float lastLinearError = 0.0f;
static volatile float lastAngularError = 0.0f;

static volatile bool collisionDetected = false;
static volatile bool mouseControlEnabled = false;
static volatile bool sideSensorsCloseControlEnabled = false;
static volatile bool sideSensorsFarControlEnabled = false;

static volatile float sideSensorsIntegral;

// TODO: Set percentage of max force.
float getMaxForce(void)
{
	return atomic_read_f32(&maxForce);
}

void setMaxForce(float value)
{
    atomic_write_f32(&maxForce, value);
}

void resetMaxForce(void)
{
    atomic_write_f32(&maxForce, MOUSE_WHEEL_MAX_FORCE_CONT_N);
}

float getLinearAcceleration(void)
{
	return 2 * getMaxForce() / MOUSE_MASS_KG;
}

float getLinearDeceleration(void)
{
	return 2 * getMaxForce() / MOUSE_MASS_KG;
}

float getTargetLinearSpeed(void)
{
	return atomic_read_f32(&targetLinearSpeed);
}

void setTargetLinearSpeed(float value)
{
    atomic_write_f32(&targetLinearSpeed, value);
}

float getIdealLinearSpeed(void)
{
	return atomic_read_f32(&idealLinearSpeed);
}

void setIdealLinearSpeed(float value)
{
    atomic_write_f32(&idealLinearSpeed, value);
}

float getIdealAngularSpeed(void)
{
	return atomic_read_f32(&idealAngularSpeed);
}

void setIdealAngularSpeed(float value)
{
    atomic_write_f32(&idealAngularSpeed, value);
}

float getMaxLinearSpeed(void)
{
	return atomic_read_f32(&maxLinearSpeed);
}

void setMaxLinearSpeed(float value)
{
    atomic_write_f32(&maxLinearSpeed, value);
}

void resetMaxLinearSpeed(void)
{
    atomic_write_f32(&maxLinearSpeed, MOUSE_WHEEL_MAX_VEL_MMPS / MILLIMETERS_PER_METER);
}

void updateIdealLinearSpeed(void)
{
	if (idealLinearSpeed < targetLinearSpeed) {
		idealLinearSpeed += getLinearAcceleration() / pidFrequency;
        
		if (idealLinearSpeed > targetLinearSpeed)
			idealLinearSpeed = targetLinearSpeed;
	} else if (idealLinearSpeed > targetLinearSpeed) {
		idealLinearSpeed -= getLinearDeceleration() / pidFrequency;
        
		if (idealLinearSpeed < targetLinearSpeed)
			idealLinearSpeed = targetLinearSpeed;
	}
}

bool sensorIsWallFront() {
    uint32_t distance = getRobotDistanceUm(SENSOR_CENTER);
    
    return distance > FRONT_WALL_DETECTION ? false : true;
}
bool sensorIsWallRight() {
    uint32_t distance = getRobotDistanceUm(SENSOR_RIGHT);
    
    return distance > SIDE_WALL_DETECTION ? false : true;
}
bool sensorIsWallLeft() {
    uint32_t distance = getRobotDistanceUm(SENSOR_LEFT);
    
    return distance > SIDE_WALL_DETECTION ? false : true;
}

float getSideSensorsCloseError(void)
{
	int32_t leftError = (int32_t) getRobotDistanceUm(SENSOR_LEFT) - MIDDLE_MAZE_DISTANCE_UM;
	int32_t rightError = (int32_t) getRobotDistanceUm(SENSOR_RIGHT) - MIDDLE_MAZE_DISTANCE_UM;

	if ((leftError > 0) && (rightError < 0))
		return (float) rightError / MICROMETERS_PER_METER;
	if ((rightError > 0) && (leftError < 0))
		return (float) -leftError / MICROMETERS_PER_METER;
    
	return 0.0f;
}

float getSideSensorsFarError(void)
{
	int32_t leftError = (int32_t) getRobotDistanceUm(SENSOR_LEFT) - MIDDLE_MAZE_DISTANCE_UM;
	int32_t rightError = (int32_t) getRobotDistanceUm(SENSOR_RIGHT) - MIDDLE_MAZE_DISTANCE_UM;

    const int32_t upperBound = 100000; // 10cm
    const int32_t lowerBound =  40000; // 4cm
    
	if ((leftError > upperBound) && (rightError < lowerBound))
		return (float) rightError / MICROMETERS_PER_METER;
	if ((rightError > upperBound) && (leftError < lowerBound))
		return (float) -leftError / MICROMETERS_PER_METER;
    
	return 0.0f;
}

static inline float ffTerm(FFProfile_t *p, float v_target)
{
    const float a_est   = (v_target - p->prev_v) * pidFrequency;   // dv/dt
    p->prev_v           = v_target;

    float ff = p->Kv * v_target + p->Ka * a_est;
    
    if (v_target >  1e-6f) ff += p->Ks;
    if (v_target < -1e-6f) ff -= p->Ks;
    return ff;
}

int16_t mouseControlStep() {
	float sideSensorsFeedback = 0.;

	if (!mouseControlEnabled)
		return 1;
    
	updateIdealLinearSpeed();

	if (sideSensorsCloseControlEnabled) {
		sideSensorsFeedback += getSideSensorsCloseError();
		sideSensorsIntegral += sideSensorsFeedback;
	}

	if (sideSensorsFarControlEnabled) {
		sideSensorsFeedback += getSideSensorsFarError();
		sideSensorsIntegral += sideSensorsFeedback;
	}
    
    updateEncoderVelocities();
    float measuredLinVel = getEncoderLinearVelocityMmPerSec() / MILLIMETERS_PER_METER;
    linearError += idealLinearSpeed - measuredLinVel;

    /*
    bool status = imuReadFifoSync();
    float measuredAngVelScaled = 0.0f;
    if (status) {
        float measuredAngVelRaw = atomic_read_f32(&rawFifoGyroMeasurements[YAW]);
        imuScaleGyroMeasurementFloat(&measuredAngVelRaw, &measuredAngVelScaled);
        measuredAngVelScaled *= DEG2RAD;
        
        angularError += idealAngularSpeed - measuredAngVelScaled;
    }
    */
    
    int16_t measuredAngVelRaw[3];
    float measuredAngVelScaled;
    
    bool status = imuReadGyroSync(measuredAngVelRaw, NULL, true);
    if (status) {
        imuScaleGyroMeasurement(&measuredAngVelRaw[YAW], &measuredAngVelScaled);
        measuredAngVelScaled *= DEG2RAD;
        
        angularError += idealAngularSpeed - measuredAngVelScaled;
    }

    float linearPower  = pidLin.Kp * linearError
                       + pidLin.Kd * (linearError - lastLinearError);

    float angularPower = pidAng.Kp      * angularError
                       + pidAng.Kd      * (angularError - lastAngularError)
                       + pidAngSens.Kp  * sideSensorsFeedback
                       + pidAngSens.Ki  * sideSensorsIntegral;
    
    const float halfWheelBase = MOUSE_WHEEL_SEPARATION_MM / (MILLIMETERS_PER_METER * 2.0f);
    
    const float ffLinear = ffTerm(&ffLin, idealLinearSpeed);
    const float ffAngular = ffTerm(&ffAng, idealAngularSpeed);
    
    const float ffLeft  = ffLinear + ffAngular;
    const float ffRight = ffLinear - ffAngular;

	float left_f  = ffLeft + linearPower + angularPower;
    if (left_f >  100.0f) left_f =  100.0f;
    if (left_f < -100.0f) left_f = -100.0f;
    int8_t powerLeft  = (int8_t) left_f;

    float right_f = ffRight + linearPower - angularPower;
    if (right_f >  100.0f) right_f =  100.0f;
    if (right_f < -100.0f) right_f = -100.0f;
    int8_t powerRight = (int8_t) right_f;

    uint8_t buffer[255];
    size_t idx = 0;
    
    {
        buffer[idx++] = FRAME_START_BYTE;

        memcpy(&buffer[idx], &targetLinearSpeed, sizeof(targetLinearSpeed));
        idx += sizeof(targetLinearSpeed);

        memcpy(&buffer[idx], &idealLinearSpeed, sizeof(idealLinearSpeed));
        idx += sizeof(idealLinearSpeed);

        memcpy(&buffer[idx], &measuredLinVel, sizeof(measuredLinVel));
        idx += sizeof(measuredLinVel);

        memcpy(&buffer[idx], &idealAngularSpeed, sizeof(idealAngularSpeed));
        idx += sizeof(idealAngularSpeed);

        memcpy(&buffer[idx], &measuredAngVelScaled, sizeof(measuredAngVelScaled));
        idx += sizeof(measuredAngVelScaled);

        memcpy(&buffer[idx], &linearError, sizeof(linearError));
        idx += sizeof(linearError);

        memcpy(&buffer[idx], &angularError, sizeof(angularError));
        idx += sizeof(angularError);
        
        memcpy(&buffer[idx], &sideSensorsFeedback, sizeof(sideSensorsFeedback));
        idx += sizeof(sideSensorsFeedback);
        
        memcpy(&buffer[idx], &sideSensorsIntegral, sizeof(sideSensorsIntegral));
        idx += sizeof(sideSensorsIntegral);
        
        memcpy(&buffer[idx], &ffLeft, sizeof(ffLeft));
        idx += sizeof(ffLeft);
        
        memcpy(&buffer[idx], &ffRight, sizeof(ffRight));
        idx += sizeof(ffRight);
        
        memcpy(&buffer[idx], &linearPower, sizeof(linearPower));
        idx += sizeof(linearPower);
        
        memcpy(&buffer[idx], &angularPower, sizeof(angularPower));
        idx += sizeof(angularPower);
        
        memcpy(&buffer[idx], &powerLeft, sizeof(powerLeft));
        idx += sizeof(powerLeft);
        
        memcpy(&buffer[idx], &powerRight, sizeof(powerRight));
        idx += sizeof(powerRight);

        buffer[idx++] = FRAME_END_BYTE;
        
        putsUART1(buffer, idx);
    }
    
    setMotorPower(MOTOR_LEFT, powerLeft);
    setMotorPower(MOTOR_RIGHT, powerRight);

	lastLinearError = linearError;
	lastAngularError = angularError;

	//if (motor_driver_saturation() >
	//    MAX_MOTOR_DRIVER_SATURATION_PERIOD * SYSTICK_FREQUENCY_HZ)
	//	set_collision_detected();
    
    return 1;
}

void sideSensorsCloseControl(bool value)
{
	sideSensorsCloseControlEnabled = value;
}

void sideSensorsFarControl(bool value)
{
	sideSensorsFarControlEnabled = value;
}

void disableWallsControl(void)
{
	sideSensorsCloseControl(false);
	sideSensorsFarControl(false);
}

void resetControlErrors(void)
{
	sideSensorsIntegral = 0;
	linearError = 0;
	angularError = 0;
	lastLinearError = 0;
	lastAngularError = 0;
}

void resetControlSpeed(void)
{
	targetLinearSpeed = 0.;
	idealLinearSpeed = 0.;
	idealAngularSpeed = 0.;
}

void resetControlAll(void)
{
    resetControlErrors();
    resetControlSpeed();
}

void enableMouseControl(void)
{
	mouseControlEnabled = true;
}

void disableMouseControl(void)
{
	mouseControlEnabled = false;
}


void initMouseController(Timer_t timer, uint16_t numTicks, float timer_hz) {
    pidFrequency = timer_hz / numTicks;
    
    const float dt_ms = 1000.0f / pidFrequency;   // real control period in ms

    // PID Gains where tuned w.r.t. 1ms ticks, therefore we scale them:
    // Linear-speed positional PD
    pidLin.Kp = MCTRL_LIN_PID_KP;
    pidLin.Ki = 0.0f;
    pidLin.Kd = MCTRL_LIN_PID_KD * pidFrequency;

    // Angular-rate positional PD
    pidAng.Kp = MCTRL_ANG_PID_KP;
    pidAng.Ki = 0.0f;
    pidAng.Kd = MCTRL_ANG_PID_KD * pidFrequency;

    // Side-sensor "wall following" loop
    pidAngSens.Kp = MCTRL_ANG_SENS_PID_KP;
    pidAngSens.Ki = MCTRL_ANG_SENS_PID_KI * dt_ms;
    pidAngSens.Kd = 0.0f;
    
    targetLinearSpeed = 0.0f;
    
    registerTimerCallback(timer, mouseControlStep, numTicks);
}