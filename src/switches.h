#ifndef SWITCHES_H
#define SWITCHES_H

/**
 * @brief Switch callback function.
 *
 * Return 0 to be unregistered, or 1 to remain registered.
 */
typedef int16_t (*SwitchCallback_t)(void);

/**
 * @brief Registers a switch callback function.
 *
 * Takes a pointer to the callback function to be called when the switch changes.
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t registerSwitchCallback(SwitchCallback_t callback);

/**
 * @brief Removes a specific switch callback function.
 *
 * Takes a pointer to the callback function to remove.
 * 
 * @return A positive value on success, negative on failure.
 */
int16_t removeSwitchCallback(SwitchCallback_t callback);

/**
 * @brief Removes all registered switch callback functions.
 */
void clearSwitchCallbacks(void);

#endif /* SWITCHES_H */
