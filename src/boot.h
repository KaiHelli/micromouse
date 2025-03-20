#ifndef BOOT_H
#define BOOT_H

/**
 * @brief Initializes the microchip on startup.
 * 
 * Sets up the device configuration and prepares hardware components as needed.
 */
void bootSetup();

/**
 * @brief Initiates a software reset of the device.
 * 
 * Resets the microcontroller through software, causing it to restart execution.
 */
void bootReset();

#endif /* BOOT_H */
