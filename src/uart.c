
#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <xc.h>

#include "IOconfig.h"
#include "pwm.h"
#include "uart.h"
#include "interrupts.h"


#define MAX_NUMBER 100 // Maximum valid number
#define MIN_NUMBER 0 // Minimum valid number

typedef enum {
    STATE_IDLE,
    STATE_READ_NUM,
} UartRxState;

static volatile char   txBuffer[UART_BUFFER_SIZE];
static volatile size_t txWriteIndex = 0;
static volatile size_t txReadIndex  = 0;
static volatile size_t txCount      = 0;

/*
 *	set-up the serial port
 *   here we aim to achieve a data transfer rate of 57.6 kbit/s,
 *   based on Fcycle=40Mhz
 *   BaudRate=Fcycle/(16*(BRG+1))
 *   ==> BRG=Fcy/(16*BaudRate) - 1 = 40000000/(16*57600) - 1 = 42.4
 *   ==> choose 42 ==> BaudRate= 58.139 kbit/s, which is ~ 0.93% off.
 *
 *
 * Python function for calculation
 * import pandas as pd
 * 
 * def calc_brgs(fcy):
 *   df = pd.DataFrame([{
 *       'Desired Baud Rate': freq,
 *       # Use walrus operator := to capture intermediate values inline
 *       'Calculated BRG': (b := (fcy/(freq*16) - 1)),
 *       'Rounded BRG': (rb := round(b)),
 *       'Actual Baud Rate': (af := (fcy/(16*(rb + 1)))),
 *       'Error (%)': 100*(af/freq - 1)
 *   } for freq in [9600*m for m in [1,2,4,6,12,24,32,48,96]]])
 *   
 *   print(df)
 *   return df
 *
 */
void setupUART1(void)
{
    U1MODEbits.UARTEN = 0; // switch the uart off during set-up
    U1BRG = 21; // baud rate register
    U1MODEbits.LPBACK = 0; // in loopback mode for test! TODO: set to no loop-back (=0) after test

    U1MODEbits.WAKE = 0; // do not wake up on serial port activity

    U1MODEbits.ABAUD = 0; // no auto baud rate detection
    U1MODEbits.PDSEL = 0; // select 8 bits data, no parity
    U1MODEbits.STSEL = 0; // one stop bit
    U1MODEbits.BRGH = 0; // No High Speed Mode

    IFS0bits.U1RXIF = 0; // reset the receive interrupt flag
    IFS0bits.U1TXIF = 0; // reset the transmission interrupt flag

    IPC2bits.U1RXIP = IP_UART_RX; // set the RX interrupt priority
    IPC3bits.U1TXIP = IP_UART_TX; // set the TX interrupt priority

    U1STAbits.URXISEL = 0; // generate a receive interrupt as soon as a character has arrived

    U1STAbits.UTXISEL1 = 0; // generate a transmit interrupt as soon as one location is empty in the transmit buffer
    U1STAbits.UTXISEL0 = 0;

    IEC0bits.U1RXIE = 1; // enable the receive interrupt
    IEC0bits.U1TXIE = 0; // disable the transmit interrupt

    // FINALLY,
    U1MODEbits.UARTEN = 1; // switch the uart on
    U1STAbits.UTXEN = 1; // enable the transmission of data; must be set after UARTEN

    //  U1MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
    //	U1STA  = 0x0440; /* Reset status register and enable TX & RX*/
}

void controlPWMCycle(char c)
{
    static char rxBuffer[32];
    static int rxIndex = 0;
    static UartRxState rxState = STATE_IDLE;

    switch (rxState) {
    case STATE_IDLE:
        if (c == '<') {
            // Start of a new command
            rxIndex = 0;
            rxState = STATE_READ_NUM;
        }
        break;

    case STATE_READ_NUM:
        if (c == '>') {
            // End of the message, process the number if within bounds
            // Null-terminate the buffer
            rxBuffer[rxIndex] = '\0';
            float value;

            // Use sscanf to parse the number (supporting both integers and floats)
            if (sscanf(rxBuffer, "%f", &value) == 1 && value >= MIN_NUMBER && value <= MAX_NUMBER) {
                setPWMDutyCycle(LED5_PWM_MODULE, LED5_PWM_CHANNEL, (1 - value));
            }
            // Reset state
            rxState = STATE_IDLE;
        } else if (rxIndex < sizeof(rxBuffer) - 1) {
            // Accumulate character if there is space in the buffer
            rxBuffer[rxIndex++] = c;
        } else {
            // Buffer overflow, discard the command
            rxState = STATE_IDLE;
        }
        break;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    uint16_t rxData; // a local buffer to copy the data into

    /**Set the UART2 receiving interrupt flag to zero*/

    IFS0bits.U1RXIF = 0;

    // we should now read out the data
    rxData = U1RXREG;
    if (txCount == 0) {
        // and copy it back out to UART
        U1TXREG = rxData;
    }

    controlPWMCycle((char)rxData);

    // we should also clear the overflow bit if it has been set (i.e. if we were to slow to read out the fifo)
    U1STAbits.OERR = 0; // we reset it all the time
    // some notes on this from the data sheet
    /*
    If the FIFO is full (four characters) and a fifth character is fully received into the UxRSR register,
    the overrun error bit, OERR (UxSTA<1>), will be set. The word in UxRSR will be kept, but further
    transfers to the receive FIFO are inhibited as long as the OERR bit is set. The user must clear
    the OERR bit in software to allow further data to be received.
    If it is desired to keep the data received prior to the overrun, the user should first read all five
    characters, then clear the OERR bit. If the five characters can be discarded, the user can simply
    clear the OERR bit. This effectively resets the receive FIFO and all prior received data is lost.

    The data in the receive FIFO should be read prior to clearing the OERR bit. The
    FIFO is reset when OERR is cleared, which causes all data in the buffer to be lost.
    */
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear the interrupt flag

    // Send as many characters as possible while there's data and TX buffer not full
    while ((txCount > 0) && (!U1STAbits.UTXBF)) {
        U1TXREG = txBuffer[txReadIndex];
        txReadIndex = (txReadIndex + 1) % UART_BUFFER_SIZE;
        txCount--;
    }

    if (txCount == 0) {
        // No more data to send
        IEC0bits.U1TXIE = 0; 
    }
}


int8_t putsUART1(char* buffer)
{
    size_t length = strlen(buffer);
    if (length == 0) {
        return 0; // Nothing to send
    }

    // If message is bigger than our entire ring buffer, treat as overflow
    if (length > UART_BUFFER_SIZE) {
        return UART_BUFFER_OVERFLOW;
    }

    // First, enable TX interrupt so the buffer can actively drain (in case it's partially full)
    IEC0bits.U1TXIE = 1;

    // Busy-wait until we have enough space for this entire message
    while ((UART_BUFFER_SIZE - txCount) < length) {
        // LED1 = ~LED1;
        // Just spin until there's space
        // The ISR will continue to drain the buffer in the background
    }

    // Now we definitely have space to copy safely;
    // briefly disable interrupts to avoid partial writes
    IEC0bits.U1TXIE = 0;

    // Copy data into the ring buffer
    for (size_t i = 0; i < length; i++) {
        txBuffer[txWriteIndex] = buffer[i];
        txWriteIndex = (txWriteIndex + 1) % UART_BUFFER_SIZE;
    }
    txCount += length;

    // Re-enable TX interrupt and trigger it
    IEC0bits.U1TXIE = 1;
    IFS0bits.U1TXIF = 1;

    return 0;
}

int8_t getUART1Status(void)
{
    /* If there's at least one byte in the buffer, UART is busy */
    return (txCount > 0) ? UART_BUSY : UART_IDLE;
}

int8_t putsUART1Sync(char* buffer)
{
    int8_t result = putsUART1(buffer); // Start the asynchronous transmission

    if (result != 0) {
        // Return immediately if the UART is busy or there's an error
        return result;
    }

    // Wait for the transmission to complete
    while (getUART1Status() == UART_BUSY) {
    }

    return 0; // Transmission completed successfully
}

void putsUART1Reference(char* buffer)
{
    char* temp_ptr = (char*)buffer;

    /* transmit till NULL character is encountered */

    if (U1MODEbits.PDSEL == 3) /* check if TX is 8bits or 9bits */
    {
        while (*buffer != '\0') {
            while (U1STAbits.UTXBF)
                ; /* wait if the buffer is full */
            U1TXREG = *buffer++; /* transfer data word to TX reg */
        }
    } else {
        while (*temp_ptr != '\0') {
            while (U1STAbits.UTXBF)
                ; /* wait if the buffer is full */
            U1TXREG = *temp_ptr++; /* transfer data byte to TX reg */
        }
    }
}