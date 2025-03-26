#ifndef SWITCHES_H
#define SWITCHES_H

#define NUM_SWITCHES 1
#define SWITCH_CALLBACK_BUFFER_SIZE 5

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SWITCH_1 = 0,
} Switch_t;

/**
 * @brief Switch callback function.
 *
 * Return 0 to be unregistered, or 1 to remain registered.
 */
typedef int16_t (*SwitchCallback_t)(void);

/**
 * @brief Initializes Switch 1 interrupts.
 */
void initSwitch1(void);

/**
 * @brief Enables or disables the interrupt for the specified switch.
 * Takes a Switch_t and a boolean state (true = enable, false = disable).
 */
void setSwitchInterruptState(Switch_t sw, bool state);

/**
 * @brief Registers a switch callback function.
 *
 * Takes a pointer to the callback function to be called when the switch changes.
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t registerSwitchCallback(Switch_t sw, SwitchCallback_t callback);

/**
 * @brief Removes a specific switch callback function.
 *
 * Takes a pointer to the callback function to remove.
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t removeSwitchCallback(Switch_t sw, SwitchCallback_t callback);

/**
 * @brief Removes all registered switch callback functions.
 */
void clearSwitchCallbacks(Switch_t sw);

#endif /* SWITCHES_H */
