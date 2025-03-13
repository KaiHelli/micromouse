/**
 * 
 * Possible improvements:
 * - Improve error handling
 * - Add watchdog code to check for stuck communication
 * 
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "i2c.h"
#include "IOconfig.h"
#include "serialComms.h"
#include "interrupts.h"

#define DEBUG 1

// -----------------------------------------------------
// I2C state machine states
// -----------------------------------------------------
typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_START,
    I2C_STATE_SEND_ADDRESS,
    I2C_STATE_SEND_DATA,
    I2C_STATE_RESTART,
    I2C_STATE_SEND_ADDRESS_R,
    I2C_STATE_READ_DATA,
    I2C_STATE_STOP,
    I2C_STATE_DONE,
    I2C_STATE_ERROR
} I2C_State_t;

// -----------------------------------------------------
// Structure for one asynchronous I2C transaction
// -----------------------------------------------------
typedef struct {
    volatile I2C_State_t state;

    uint8_t  devAddr;  // 7-bit slave address (not shifted)
    
    // Write data
    const uint8_t *writeData;
    uint8_t  writeLen;
    uint8_t  writeIndex;
    
    // Read data
    uint8_t *readData;
    uint8_t  readLen;
    uint8_t  readIndex;
    
    // Callback to call when done or error
    void (*callback)(bool success);
} I2C_Transaction_t;


// -----------------------------------------------------
// I2C Buffer Definitions
// -----------------------------------------------------
static volatile I2C_Transaction_t i2cBuffer[I2C1_BUFFER_SIZE];
static volatile uint8_t i2cBufferReadIndex = 0;
static volatile uint8_t i2cBufferWriteIndex = 0;
static volatile uint8_t i2cBufferCount = 0;

// This flag indicates whether a transaction is in progress on the bus
static volatile bool i2cInProgress = false;


// Global transaction for I2C1 (single at a time)
static volatile I2C_Transaction_t i2cTransaction;

/**
 *  Setup the I2C port (example: 400 kHz @ Fcy=40 MHz)
 * 
 *  I2CXBRG = ((1/fscl - delay) * fcy) - 2
 *  delay typically 110 - 130ns
 * 
 *  I2CXBRG = ((1/400kHz - 120ns) * 40MHz) - 2 = 93.2
 *  => use 93 (delay = 125ns)
 * 
 */
void setupI2C1(void)
{
    // Disable module for configuration
    I2C1CONbits.I2CEN = 0;
    
    // Basic configuration
    I2C1CONbits.I2CSIDL = 1;  // Stop module in Idle mode
    I2C1CONbits.A10M    = 0;  // 7-bit addressing
    I2C1CONbits.DISSLW  = 0;  // Disable slew rate control
    I2C1CONbits.SMEN    = 0;  // Disable SMBus input thresholds
    I2C1CONbits.GCEN    = 0;  // Disable general call
    I2C1CONbits.STREN   = 0;  // Disable clock stretching
    
    // Baud rate for ~400 kHz
    I2C1BRG = 93;  
    
    // Clear & enable master interrupt
    IFS1bits.MI2C1IF = 0;
    IEC1bits.MI2C1IE = 0;
    IPC4bits.MI2C1IP = IP_I2C;
    
    // Enable module
    I2C1CONbits.I2CEN = 1;
}

// -----------------------------------------------------
// Helper Function: Start next transaction from buffer
// -----------------------------------------------------
static void startNextI2CTransaction(void)
{
    // If no transactions in the buffer, mark bus as free and return
    if (i2cBufferCount == 0)
    {
        i2cInProgress = false;
        return;
    }

    // Pop the next transaction off the ring buffer
    i2cTransaction = i2cBuffer[i2cBufferReadIndex];
    i2cBufferReadIndex = (i2cBufferReadIndex + 1) % I2C1_BUFFER_SIZE;
    i2cBufferCount--;

    // Move to START state
    i2cTransaction.state = I2C_STATE_START;

    // Initiate a START condition
    i2cInProgress = true;
    I2C1CONbits.SEN = 1;
}



bool getI2C1Status(void) {
    return i2cTransaction.state == I2C_STATE_IDLE ? I2C_IDLE : I2C_BUSY;
}

/**
 * Begins an asynchronous I2C transfer.
 * 
 * @param devAddr  7-bit slave address (no R/W bit).
 * @param wData    Pointer to data to write (or NULL if none).
 * @param wLen     Number of bytes to write.
 * @param rData    Pointer to buffer for reading (or NULL if none).
 * @param rLen     Number of bytes to read.
 * @param cb       Callback function when done or on error.
 * @return true if started successfully, false if I2C1 is busy.
 */
bool putsI2C1(uint8_t devAddr, const uint8_t *wData, uint8_t wLen,
              uint8_t *rData, uint8_t rLen, void (*cb)(bool))
{
    // Wait actively if buffer is full.
    // As interrupts complete transactions, the buffer frees up in the background.
    while (i2cBufferCount >= I2C1_BUFFER_SIZE)
    {
        // LED2 = ~LED2;
        // Optionally, you can place CLRWDT() or a small delay here to avoid watchdog resets
        // while actively waiting.
    }
    
    // If there's truly nothing to do, succeed immediately
    if ((wLen == 0) && (rLen == 0))
    {
        if (cb) {
            cb(true);
        }
        return 0;
    }

    // Prepare a new transaction
    I2C_Transaction_t newTrans;
    newTrans.devAddr    = devAddr;
    newTrans.writeData  = wData;
    newTrans.writeLen   = wLen;
    newTrans.writeIndex = 0;
    newTrans.readData   = rData;
    newTrans.readLen    = rLen;
    newTrans.readIndex  = 0;
    newTrans.callback   = cb;
    newTrans.state      = I2C_STATE_IDLE;  // Will set START just before sending

    // ---- CRITICAL SECTION for buffer manipulation ----
    IEC1bits.MI2C1IE = 0;
    
    // Push the transaction into the buffer
    i2cBuffer[i2cBufferWriteIndex] = newTrans;
    i2cBufferWriteIndex = (i2cBufferWriteIndex + 1) % I2C1_BUFFER_SIZE;
    i2cBufferCount++;

    // If bus is not currently in a transaction, start immediately
    if (!i2cInProgress)
    {
        startNextI2CTransaction();
    }
    
    IEC1bits.MI2C1IE = 1;
    // ---- END CRITICAL SECTION ----

    return 0; // success
}

/**
 * Synchronous (blocking) version of putsI2C1. 
 * No callback is needed. It returns:
 *   - true, if the transaction succeeded
 *   - false, if the bus was busy or any error occurred
 */
bool putsI2C1Sync(uint8_t devAddr, const uint8_t *wData, uint8_t wLen, uint8_t *rData, uint8_t rLen)
{
    int8_t result = putsI2C1(devAddr, wData, wLen, rData, rLen, NULL); // Start the asynchronous transmission

    if (result != 0) {
        // Return immediately if the I2C is busy or there's an error
        return result;
    }

    // Wait for the operation to complete
    while (getI2C1Status() == I2C_BUSY) {
    }

    return 0; // Operation completed successfully
}

// -----------------------------------------------------
// Master I2C interrupt service routine
// -----------------------------------------------------
void __attribute__((__interrupt__, auto_psv)) _MI2C1Interrupt(void)
{
    // Clear master interrupt flag
    IFS1bits.MI2C1IF = 0;

    switch (i2cTransaction.state)
    {
        // -------------------------------------------------
        // 1) START
        // -------------------------------------------------
        case I2C_STATE_START:
            
            if (!I2C1CONbits.SEN) // once START is done
            {
                if (i2cTransaction.writeLen > 0)
                {
                    // Write first => address with R/W=0
                    i2cTransaction.state = I2C_STATE_SEND_ADDRESS;
                    I2C1TRN = (i2cTransaction.devAddr << 1) | 0;
                }
                else
                {
                    // Pure read => address with R/W=1
                    i2cTransaction.state = I2C_STATE_SEND_ADDRESS_R;
                    I2C1TRN = (i2cTransaction.devAddr << 1) | 1;
                }
            }
            break;

        // -------------------------------------------------
        // 2) SEND_ADDRESS (Write bit = 0)
        // -------------------------------------------------
        case I2C_STATE_SEND_ADDRESS:
            
            if (!I2C1STATbits.TRSTAT)  // transmission done?
            {
                if (I2C1STATbits.ACKSTAT == 1) {
                    // NACK => error
                    i2cTransaction.state = I2C_STATE_ERROR;
                    IFS1bits.MI2C1IF = 1; // force another interrupt
                } else {
                    // ACK => send data if any
                    if (i2cTransaction.writeLen > 0) {
                        i2cTransaction.state = I2C_STATE_SEND_DATA;
                        I2C1TRN = i2cTransaction.writeData[i2cTransaction.writeIndex++];
                    }
                }
            }
            break;

        // -------------------------------------------------
        // 3) SEND_DATA (writing)
        // -------------------------------------------------
        case I2C_STATE_SEND_DATA:
            
            if (!I2C1STATbits.TRSTAT) 
            {
                if (I2C1STATbits.ACKSTAT == 1) {
                    // NACK => error
                    i2cTransaction.state = I2C_STATE_ERROR;
                    IFS1bits.MI2C1IF = 1; // force another interrupt
                } else {
                    // Byte was ACKed
                    if (i2cTransaction.writeIndex < i2cTransaction.writeLen) {
                        // More data to send
                        I2C1TRN = i2cTransaction.writeData[i2cTransaction.writeIndex++];
                    }
                    else if (i2cTransaction.readLen > 0) {
                        // Done writing, now read => repeated start
                        i2cTransaction.state = I2C_STATE_RESTART;
                        I2C1CONbits.RSEN = 1;
                    } else {
                        // Done writing, no read => stop
                        i2cTransaction.state = I2C_STATE_STOP;
                        I2C1CONbits.PEN = 1;
                    }
                }
            }
            break;

        // -------------------------------------------------
        // 4) RESTART
        // -------------------------------------------------
        case I2C_STATE_RESTART:
            
            if (!I2C1CONbits.RSEN) // repeated start done
            {
                // Address with R/W=1 (read)
                i2cTransaction.state = I2C_STATE_SEND_ADDRESS_R;
                I2C1TRN = (i2cTransaction.devAddr << 1) | 1;
            }
            break;

        // -------------------------------------------------
        // 5) SEND_ADDRESS_R (Read bit = 1)
        // -------------------------------------------------
        case I2C_STATE_SEND_ADDRESS_R:
            
            if (!I2C1STATbits.TRSTAT)
            {
                if (I2C1STATbits.ACKSTAT == 1) {
                    // NACK => error
                    i2cTransaction.state = I2C_STATE_ERROR;
                    IFS1bits.MI2C1IF = 1; // force another interrupt
                } else {
                    // ACK => start reading
                    i2cTransaction.state = I2C_STATE_READ_DATA;
                    I2C1CONbits.RCEN = 1; // enable receive
                }
            }
            break;

        // -------------------------------------------------
        // 6) READ_DATA
        // -------------------------------------------------
        case I2C_STATE_READ_DATA:
            
            if (I2C1STATbits.RBF)
            {
                // Store the byte
                i2cTransaction.readData[i2cTransaction.readIndex++] = I2C1RCV;

                // Check if more to read
                if (i2cTransaction.readIndex < i2cTransaction.readLen)
                {
                    // More => ACK=0
                    I2C1CONbits.ACKDT = 0; // 0=ACK
                    I2C1CONbits.ACKEN = 1;
                }
                else
                {
                    // Done => NACK=1
                    I2C1CONbits.ACKDT = 1;
                    I2C1CONbits.ACKEN = 1;
                }
            }
            // After ACK/NACK is sent, hardware clears ACKEN
            else if (!I2C1CONbits.ACKEN && !I2C1CONbits.RCEN)
            {
                // If we still have more to read, enable RCEN again
                if (i2cTransaction.readIndex < i2cTransaction.readLen)
                {
                    I2C1CONbits.RCEN = 1;
                }
                else
                {
                    // Done reading => stop
                    i2cTransaction.state = I2C_STATE_STOP;
                    I2C1CONbits.PEN = 1;
                }
            }
            break;

        // -------------------------------------------------
        // 7) STOP
        // -------------------------------------------------
        case I2C_STATE_STOP:
            
            if (!I2C1CONbits.PEN) // stop done
            {
                i2cTransaction.state = I2C_STATE_DONE;
                IFS1bits.MI2C1IF = 1; // force another interrupt
            }
            break;

        // -------------------------------------------------
        // 8) DONE
        // -------------------------------------------------
        case I2C_STATE_DONE:
            if (i2cTransaction.callback) {
                i2cTransaction.callback(true);
            }
            i2cTransaction.state = I2C_STATE_IDLE;
            // Start next transaction if pending in buffer
            startNextI2CTransaction();
            break;

        // -------------------------------------------------
        // 9) ERROR
        // -------------------------------------------------
        case I2C_STATE_ERROR:
            if (!I2C1CONbits.PEN && !I2C1STATbits.P) {
                I2C1CONbits.PEN = 1; // Attempt STOP on error
            }
            if (i2cTransaction.callback) {
                i2cTransaction.callback(false);
            }
            i2cTransaction.state = I2C_STATE_IDLE;
            // Start next transaction if pending in buffer
            startNextI2CTransaction();
            break;

        // -------------------------------------------------
        // Default / IDLE (shouldn't happen in non-error cases)
        // -------------------------------------------------
        default:
            break;
    }
}
