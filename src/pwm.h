#ifndef PWM_H
#define PWM_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdint.h>
#include <stdbool.h>
#include "clock.h"

/**
 * @brief Configures and initializes the PWM1 module.
 * 
 * This function sets up the PWM1 module and any required registers for
 * basic operation.
 */
void setupPWM1();

/**
 * @brief Configures and initializes the PWM2 module.
 * 
 * This function sets up the PWM2 module and any required registers for
 * basic operation.
 */
void setupPWM2();

/**
 * @brief Sets the PWM frequency for a specified module.
 * 
 * Takes the PWM module index and a desired frequency in Hz. Returns 0 on
 * success or a negative error code on failure.
 */
int8_t setPWMFrequency(uint8_t pwmModule, uint32_t desiredFreq);

/**
 * @brief Sets the duty cycle for a specified PWM module channel.
 * 
 * Takes the PWM module index, the channel index, and a fraction
 * (0.0 to 1.0) representing the desired duty cycle. Returns 0 on
 * success or a negative error code on failure.
 */
int8_t setPWMDutyCycle(uint8_t pwmModule, uint8_t channel, float fraction);

/**
 * @brief Enables or disables a specified PWM module channel.
 * 
 * Takes the PWM module index, the channel index, and a boolean state.
 * Returns 0 on success or a negative error code on failure.
 */
int8_t setPWMState(uint8_t pwmModule, uint8_t channel, bool state);

#endif /* PWM_H */
