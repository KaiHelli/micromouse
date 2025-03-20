#ifndef MOTORS_H
#define	MOTORS_H

#define PWM_MOTOR_MAX_DC (6 / 8.4) // voltage ranges from 2*3.7V = 7.4V to 2*4.2V = 8.4V

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
} Motor_t;

void setMotorsStandbyState(bool state);
void setMotorsBrake(bool state);
void setMotorsDirection(bool cw);
void steerMotors(int8_t steering); 

void setMotorPower(Motor_t motor, float powerInPercent);

#endif	/* MOTORS_H */

