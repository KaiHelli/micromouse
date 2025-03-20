#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

#define I2C1_BUFFER_SIZE 32 // Number of I2C transmissions that can be queued in a buffer

#define I2C_IDLE 0 // I2C is idle, ready for a new transmission
#define I2C_BUSY 1 // I2C buffer is full and thus busy transmitting

/**
 * @brief Callback function used by asynchronous I2C operations. The bool parameter
 * indicates success (true) or failure (false).
 */
typedef void (*I2CCallback_t)(bool);

/**
 * @brief Configures and initializes the I2C1 module. Sets up the I2C1 pins,
 * clock, and any required registers for I2C communication.
 */
void setupI2C1(void);

/**
 * @brief Retrieves the current status of I2C1. Returns I2C_IDLE if idle,
 * I2C_BUSY if busy.
 */
bool getI2C1Status(void);

/**
 * @brief Initiates an asynchronous I2C write/read operation on I2C1. Takes a
 * 7-bit device address, pointers for write/read data, lengths for each, and a
 * callback function. While the buffer is full and thus busy, this function
 * blocks until the transmission could be queued.
 */
void putsI2C1(uint8_t devAddr, const uint8_t *wData, uint8_t wLen,
              uint8_t *rData, uint8_t rLen, I2CCallback_t callback);

/**
 * @brief Performs a synchronous I2C write/read operation on I2C1. Takes a 7-bit
 * device address, pointers for write/read data, and lengths for each. This
 * function blocks until the operation completes. Returns true on success,
 * or false otherwise.
 */
bool putsI2C1Sync(uint8_t devAddr, const uint8_t *wData, uint8_t wLen,
                  uint8_t *rData, uint8_t rLen);

#endif /* I2C_H */
