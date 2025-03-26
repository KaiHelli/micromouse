#ifndef MOTORS_H
#define MOTORS_H
#include "pwm.h"
#include "IOconfig.h"

#define PWM_MOTOR_MAX_DC (6 / 8.4) // voltage ranges from 2*3.7V = 7.4V to 2*4.2V = 8.4V
#define SET_MOTOR_RIGHT(powerInPercent) setPWMDutyCycle(MB_PWM_MODULE, MB_PWM_CHANNEL, (powerInPercent/100.0f)*PWM_MOTOR_MAX_DC)
#define SET_MOTOR_LEFT(powerInPercent) setPWMDutyCycle(MA_PWM_MODULE, MA_PWM_CHANNEL, (powerInPercent/100.0f)*PWM_MOTOR_MAX_DC)
#define H 1
#define L 0

typedef enum {
    MOTORS_STANDBY = 0,
    MOTORS_BRAKE,
    MOTORS_BACKWARD,
    MOTORS_FORWARD,
    MOTORS_ROTATE_LEFT,
    MOTORS_ROTATE_RIGHT
} MotorState_t;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_t;

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
 * as a percentage of the motor's maximum (0.0 for no power, 100.0 for full power).
 * The steering adjustments are then applied on top of this base power.
 */
void steerMotors(int8_t steering, float powerInPercent);

/**
 * @brief Rotates the robot by the specified angle in degrees on the spot.
 * Takes an int16_t degrees value; positive rotates CW, negative CCW.
 */
void turnDegrees(int16_t degrees);

/**
 * @brief Sets the power level for a specific motor. Takes a Motor_t identifier
 * (MOTOR_LEFT or MOTOR_RIGHT) and a float percentage for speed control.
 */
void setMotorPower(Motor_t motor, float powerInPercent);

#endif /* MOTORS_H */
