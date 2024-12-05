
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "serialComms.h"
#include "IOconfig.h"
#include "myPWM.h"



volatile char txBuffer[UART_BUFFER_SIZE]; // Internal transmit buffer
volatile int8_t txIndex = 0;                 // Index of the current character being sent
volatile int8_t txLength = 0;                // Length of the data in the buffer
volatile int8_t txInProgress = 0;            // Flag indicating if a transmission is in progress

#define MAX_NUMBER 100     // Maximum valid number
#define MIN_NUMBER 0       // Minimum valid number

typedef enum {
    STATE_IDLE,
    STATE_READ_NUM,
} RxState;

/*
 *	set-up the serial port
 *   here we aim to achieve a data transfer rate of 57.6 kbit/s,
 *   based on Fcycle=26.6666Mhz 
 *   BaudRate=Fcycle/(16*(BRG+1))
 *   ==> BRG=Fcy/(16*BaudRate) - 1 = 26.666Mhz/(16*57600) - 1 = 28.23
 *   ==> choose 28 ==> BaudRate= 57.474  kbit/s, which is ~ 1% off.
 * 
 * for standard communication speed of 9600 kbit/s
 * choose 173 (factor 6)
 * 
 * For Fcycle=26.726400Mhz
 * BRG = 28 => BaudRate = 57.6 kbit/s, perfect match.
 * 
 */
void setupUART1(void)
{
	U1MODEbits.UARTEN=0; //switch the uart off during set-up
	U1BRG=28; // baud rate register
	U1MODEbits.LPBACK=0; // in loopback mode for test! TODO: set to no loop-back (=0) after test 
	
	U1MODEbits.WAKE=0; //do not wake up on serial port activity

	U1MODEbits.ABAUD=0; //no auto baud rate detection
	U1MODEbits.PDSEL=0; //select 8 bits data, no parity
	U1MODEbits.STSEL=0; //one stop bit
    U1MODEbits.BRGH = 0; // No High Speed Mode


	IFS0bits.U1RXIF=0; //reset the receive interrupt flag
	IFS0bits.U1TXIF=0; //reset the transmission interrupt flag
    
	IPC2bits.U1RXIP=3; //set the RX interrupt priority
	IPC3bits.U1TXIP=5; //set the TX interrupt priority

	U1STAbits.URXISEL=0; //generate a receive interrupt as soon as a character has arrived
	U1STAbits.UTXEN=1; //enable the transmission of data
    
    U1STAbits.UTXISEL1=0; // generate a transmit interrupt as soon as one location is empty in the transmit buffer
    U1STAbits.UTXISEL0=0; 
    
    
	IEC0bits.U1RXIE=1; //enable the receive interrupt
	IEC0bits.U1TXIE=0; //disable the transmit interrupt

	//FINALLY, 
	U1MODEbits.UARTEN=1; //switch the uart on

  	U1STAbits.UTXEN=1; //enable transmission
    
    //  U1MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
    //	U1STA  = 0x0440; /* Reset status register and enable TX & RX*/
}


void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{	
	uint16_t rxData; // a local buffer to copy the data into

	/**Set the UART2 receiving interrupt flag to zero*/
 
	IFS0bits.U1RXIF=0;

	//we should now read out the data
	rxData=U1RXREG;
    if (!txInProgress) {
        //and copy it back out to UART
        U1TXREG=rxData;
    }

    //controlLED(rxData);
    controlPWMCycle((char) rxData);
        
	//we should also clear the overflow bit if it has been set (i.e. if we were to slow to read out the fifo)
	U1STAbits.OERR=0; //we reset it all the time
	//some notes on this from the data sheet
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

void controlPWMCycle(char c)
{
    static char rxBuffer[32];
    static int rxIndex = 0;
    static RxState rxState = STATE_IDLE;

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
                    P1DC1 = (1 - value / 100) * MYPWM_MAX;
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


void controlLED(uint16_t rxData) {                                                                                                                                                                        
    // aa-----s
    // aa = 0 / 1 / 2 / 3 -> addresses LED
    // s = 0 / 1 -> state off / on
    uint8_t ledAddress = rxData >> 6;
    uint8_t ledState = rxData & 1;
    
    switch (ledAddress) {
        case 0:
            LED4 = ledState;
            break;
        case 1:
            LED5 = ledState;
            break;
        case 2:
            LED6 = ledState;
            break;
        case 3:
            LED7 = ledState;
            break;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear the interrupt flag

    if (txIndex < txLength) {
        // Send the next character
        U1TXREG = txBuffer[txIndex++];
    } else {
        // All characters are sent; disable interrupt and reset state
        txInProgress = 0;
        IEC0bits.U1TXIE = 0;
    }
}

int8_t putsUART1(char *buffer)
{
    // Disable UART Transmit Interrupt to prevent race conditions
    IEC0bits.U1TXIE = 0;
    
    // Check if a transmission is already in progress
    if (txInProgress) {
        IEC0bits.U1TXIE = 1;
        return UART_BUFFER_BUSY; // Indicate that the UART is busy
    }
    
    // Check if the UART is configured for 8-bit data
    if (U1MODEbits.PDSEL == 3) {
        IEC0bits.U1TXIE = 1;
        return UART_UNSUPPORTED_MODE; // Error: 9-bit data mode not supported
    }

    // Calculate the length of the input buffer
    size_t length = strlen(buffer);
    if (length >= UART_BUFFER_SIZE) {
        IEC0bits.U1TXIE = 1;
        return UART_BUFFER_OVERFLOW; // Error: Buffer overflow
    }
    
    // In case there is nothing to send, return.
    if (length == 0) {
        IEC0bits.U1TXIE = 1;
        return 0; // Nothing to send
    }

    // Copy data to the internal buffer
    strncpy((char *) txBuffer, buffer, length);
    
    txLength = length;
    txIndex = 0;
    txInProgress = 1;
    
    // Re-enable UART Transmit Interrupt
    // This will persistently trigger the UART IF as long as there is a spot in
    // the FIFO buffer.
    IEC0bits.U1TXIE = 1;

    // Indicate success
    return 0;
}

int8_t getUART1Status(void)
{
    return txInProgress ? UART_BUSY : UART_IDLE;
}


int8_t putsUART1Sync(char *buffer)
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


void putsUART1Reference(char *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    if(U1MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer != '\0') 
        {
            while(U1STAbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
    else
    {
        while(*temp_ptr != '\0')
        {
            while(U1STAbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
}