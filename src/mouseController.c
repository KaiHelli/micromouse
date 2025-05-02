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

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#define SIDE_WALL_DETECTION (CELL_DIMENSION_MM * 0.90)
// #define FRONT_WALL_DETECTION (CELL_DIMENSION_MM * 1.5)
#define FRONT_WALL_DETECTION (CELL_DIMENSION_MM * 1.1)

#define SENSOR_OFFSET_THRESHOLD 1 // TODO

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

// NEW STUFF

#define TICK_IN_MS 5.333f

/*
#define MCTRL_LIN_PID_KP                8.0f
#define MCTRL_LIN_PID_KD                16.0f
#define MCTRL_ANG_PID_KP                0.05f
#define MCTRL_ANG_PID_KD                1.0f
#define MCTRL_ANG_SENS_PID_KP           2.0f
#define MCTRL_ANG_SENS_PID_KI           4.0f
*/

#define MCTRL_LIN_PID_KP                (8.0f * TICK_IN_MS)  // 3.25f
#define MCTRL_LIN_PID_KD                (20.0f * TICK_IN_MS) // 16.0f // 24.0f
#define MCTRL_ANG_PID_KP                (0.13f * TICK_IN_MS)
#define MCTRL_ANG_PID_KD                (0.5f * TICK_IN_MS)
#define MCTRL_ANG_SENS_PID_KP           (2.0f * TICK_IN_MS)
#define MCTRL_ANG_SENS_PID_KI           (4.0f * TICK_IN_MS)

static volatile float maxForce = MOUSE_WHEEL_MAX_FORCE_CONT_N;
static volatile float maxLinearSpeed = MOUSE_WHEEL_MAX_VEL_MMPS;

static volatile float pidFrequency;

static volatile float targetLinearSpeed;
static volatile float idealLinearSpeed;
static volatile float idealAngularSpeed;

static volatile float linearError;
static volatile float angularError;
static volatile float lastLinearError;
static volatile float lastAngularError;

static volatile bool collisionDetected;
static volatile bool mouseControlEnabled;
static volatile bool sideSensorsCloseControlEnabled;
static volatile bool sideSensorsFarControlEnabled;

static volatile float sideSensorsIntegral;

// TODO: Set percentage of max force.
float getMaxForce(void)
{
	return maxForce;
}

void setMaxForce(float value)
{
	maxForce = value;
}

void resetMaxForce(void)
{
    maxForce = MOUSE_WHEEL_MAX_FORCE_CONT_N;
}

float getLinearAcceleration(void)
{
	return 2 * maxForce / MOUSE_MASS_KG;
}

float getLinearDeceleration(void)
{
	return 2 * maxForce / MOUSE_MASS_KG;
}

float getTargetLinearSpeed(void)
{
	return targetLinearSpeed;
}

void setTargetLinearSpeed(float value)
{
	targetLinearSpeed = value;
}

float getIdealLinearSpeed(void)
{
	return idealLinearSpeed;
}

void setIdealLinearSpeed(float value)
{
	idealLinearSpeed = value;
}

float getIdealAngularSpeed(void)
{
	return idealAngularSpeed;
}

void setIdealAngularSpeed(float value)
{
	idealAngularSpeed = value;
}

float getMaxLinearSpeed(void)
{
	return maxLinearSpeed;
}

void setMaxLinearSpeed(float value)
{
	maxLinearSpeed = value;
}

void resetMaxLinearSpeed(float value)
{
    maxLinearSpeed = MOUSE_WHEEL_MAX_VEL_MMPS;
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
    uint16_t distance = getRobotDistanceMm(SENSOR_CENTER);
    
    return distance > SIDE_WALL_DETECTION ? false : true;
}
bool sensorIsWallRight() {
    uint16_t distance = getRobotDistanceMm(SENSOR_RIGHT);
    
    return distance > SIDE_WALL_DETECTION ? false : true;
}
bool sensorIsWallLeft() {
    uint16_t distance = getRobotDistanceMm(SENSOR_LEFT);
    
    return distance > FRONT_WALL_DETECTION ? false : true;
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

	if ((leftError > 10) && (rightError < 4))
		return (float) rightError / MICROMETERS_PER_METER;
	if ((rightError > 10) && (leftError < 4))
		return (float) -leftError / MICROMETERS_PER_METER;
    
	return 0.0f;
}

// TODO: Call update encoder velocities.
// TODO: Call get gyro turn rate 

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

    float measuredLinVel = getEncoderLinearVelocityMmPerSec() / MILLIMETERS_PER_METER;
    float measuredAngVel;
    imuScaleGyroMeasurementFloat(&rawFifoGyroMeasurements[YAW], &measuredAngVel);
    measuredAngVel *= DEG2RAD;
    
	linearError += idealLinearSpeed - measuredLinVel;
	angularError += idealAngularSpeed - measuredAngVel;

	float linearPower = MCTRL_LIN_PID_KP * linearError + MCTRL_LIN_PID_KD * (linearError - lastLinearError);
	float angularPower =
	    MCTRL_ANG_PID_KP * angularError +
	    MCTRL_ANG_PID_KD * (angularError - lastAngularError) +
	    MCTRL_ANG_SENS_PID_KP * sideSensorsFeedback +
	    MCTRL_ANG_SENS_PID_KI * sideSensorsIntegral;

	float left_f  = linearPower + angularPower;
    if (left_f >  100.0f) left_f =  100.0f;
    if (left_f < -100.0f) left_f = -100.0f;
    int8_t powerLeft  = (int8_t) left_f;

    float right_f = linearPower - angularPower;
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

        memcpy(&buffer[idx], &measuredAngVel, sizeof(measuredAngVel));
        idx += sizeof(measuredAngVel);

        memcpy(&buffer[idx], &linearError, sizeof(linearError));
        idx += sizeof(linearError);

        memcpy(&buffer[idx], &angularError, sizeof(angularError));
        idx += sizeof(angularError);
        
        memcpy(&buffer[idx], &sideSensorsFeedback, sizeof(sideSensorsFeedback));
        idx += sizeof(sideSensorsFeedback);
        
        memcpy(&buffer[idx], &sideSensorsIntegral, sizeof(sideSensorsIntegral));
        idx += sizeof(sideSensorsIntegral);
        
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
    pidFrequency = timer_hz;
    
    registerTimerCallback(timer, mouseControlStep, numTicks);
}