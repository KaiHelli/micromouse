#ifndef MOTORS_H
#define MOTORS_H
#include "pwm.h"
#include "IOconfig.h"
#include "timers.h"

static const uint16_t PWM_MOTOR_MAX_DC = (60U * 100U) / 84U;  // voltage ranges from 2*3.7V = 7.4V to 2*4.2V = 8.4V

#define SET_MOTOR_RIGHT(powerInPercent) setPWMDutyCycle(MB_PWM_MODULE, MB_PWM_CHANNEL, (uint8_t)(((uint16_t)(powerInPercent) * PWM_MOTOR_MAX_DC) / 100))
#define SET_MOTOR_LEFT(powerInPercent) setPWMDutyCycle(MA_PWM_MODULE, MA_PWM_CHANNEL, (uint8_t)(((uint16_t)(powerInPercent) * PWM_MOTOR_MAX_DC) / 100))
#define H 1
#define L 0

typedef enum {
    MOTORS_BRAKE,
    MOTORS_BACKWARD,
    MOTORS_FORWARD,
    MOTORS_ROTATE_LEFT,
    MOTORS_ROTATE_RIGHT
} MotorState_t;

typedef enum {
    MOTOR_LEFT = 0, // Motor A
    MOTOR_RIGHT     // Motor B
} Motor_t;

void initMotorsState(void);
    
void setMotorsStandbyState(bool state);

void toggleMotorsStandby(void);

/**
 * @brief Puts the motors into the specified state. Takes a MotorState_t
 * (MOTORS_STANDBY, MOTORS_BRAKE, MOTORS_CW, or MOTORS_CCW) to select the mode.
 */
void setMotorsState(MotorState_t state);

/**
 * @brief Adjusts the steering by distributing power differently between motors.
 * Takes an int8_t steering value; sign and magnitude affect the power split.
 *  -100 -> Turn on spot CCW
 *   0   -> Drive straight
 *  100  -> Turn on spot CW
 * Intermediate values produce partial turning effects (e.g., slight left turn).
 * 
 * The powerInPercent parameter (float) defines the overall base power level
 * as a percentage of the motor's maximum (0 for no power, 100 for full power).
 * The steering adjustments are then applied on top of this base power.
 */
void steerMotors(int8_t steering, uint8_t powerInPercent);

/**
 * @brief Rotates the robot by the specified angle in degrees on the spot.
 * Takes an int16_t degrees value; positive rotates CW, negative CCW.
 * The powerInPercent parameter (float) defines the overall base power level
 * as a percentage of the motor's maximum (0 for no power, 100 for full power).
 * Registers a timer callback on the stated timer to check for turn completion
 * periodically.
 */
void turnDegrees(Timer_t timer, int16_t degrees, uint8_t powerInPercent);

/**
 * @brief Sets the power level for a specific motor. Takes a Motor_t identifier
 * (MOTOR_LEFT or MOTOR_RIGHT) and a integer percentage for speed control.
 */
void setMotorPower(Motor_t motor, int8_t powerInPercent);

/**
 * @brief Sets the turning direction for a specific motor. Takes a Motor_t identifier
 * (MOTOR_LEFT or MOTOR_RIGHT) and a bool forward for specifying if the motor turns forwards or backwards.
 */
void setMotorDirection(Motor_t motor, bool forward);

#endif /* MOTORS_H */
