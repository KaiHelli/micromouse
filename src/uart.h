#ifndef UART_H
#define UART_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdio.h>

#define FRAME_START_BYTE  0x03
#define FRAME_END_BYTE    ((uint8_t)~0x03)

/**
 * @brief Configures and initializes the UART1 module.
 */
void setupUART1(void);

int8_t putsUART1Str(char* buffer);

/**
 * @brief Sends a null-terminated string via UART1 asynchronously. Takes a
 * pointer to the string buffer. Returns 0 on success, or a negative error code.
 * This function blocks if the internal buffer is full until the message can be
 * queued.
 */
int8_t putsUART1(char* buffer, size_t length);

/**
 * @brief Retrieves the current status of UART1. Returns UART_IDLE if idle,
 * or UART_BUSY if buffer is full.
 */
int8_t getUART1Status(void);

int8_t putsUART1StrSync(char* buffer);

/**
 * @brief Sends a null-terminated string via UART1 synchronously. Takes a pointer
 * to the string buffer. This function blocks until transmission is complete.
 * Returns 0 on success, or a negative error code.
 */
int8_t putsUART1Sync(char* buffer, size_t length);

/**
 * @brief Reference implementation of sending a null-terminated string via UART1
 * synchronously.
 */
void putsUART1Reference(char* buffer);

#define UART_BUFFER_SIZE 2048 // Define the size of the internal buffer
#define UART_BUFFER_BUSY -1   // Error code for buffer busy
#define UART_BUFFER_OVERFLOW -2 // Error code for buffer overflow
#define UART_UNSUPPORTED_MODE -3 // Error code for unsupported modes like 9-bit data

#define UART_IDLE 0 // UART is idle, ready for a new transmission
#define UART_BUSY 1 // UART is busy transmitting

#endif /* UART_H */
