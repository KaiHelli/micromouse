#include <xc.h>

/*
 *	set-up the serial port
 *   here we aim to achieve a data transfer rate of 400 kHz,
 *   based on Fcycle=40MHz
 * 
 *  I2CXBRG = ((1/fscl - delay) * fcy) - 2
 *  delay typically 110 - 130ns
 * 
 *  I2CXBRG = ((1/400kHz - 120ns) * 40MHz) - 2 = 93.2
 *  => use 93 (delay = 125ns)
 * 
 */
void setupI2C(void) {
    // disable for I2C setup
    I2C1CONbits.I2CEN = 0;
    
    I2C1CONbits.I2CSIDL = 1;    // stop I2C in idle mode
    I2C1CONbits.SCLREL = 1;     // automatically release SCL
    I2C1CONbits.IPMIEN = 0;     // we don't use IPMI
    I2C1CONbits.A10M = 0;       // use 7-bit slave addressing
    I2C1CONbits.DISSLW = 0;     // disable slew rate control
    I2C1CONbits.SMEN = 0;       // disable SMbus input thresholds
    I2C1CONbits.GCEN = 0;       // general call address disabled
    I2C1CONbits.STREN = 0;      // disable clock stretching
    I2C1CONbits.ACKDT = 0;      // in case enabled, send ACK after master receive
    I2C1CONbits.ACKEN = 0;      // disable sending ACK after master receive
    
    
    I2C1CONbits.RCEN = 0;       // init I2C receive mode
    I2C1CONbits.PEN = 0;        // init I2C stop condition
    I2C1CONbits.RSEN = 0;       // init I2C repeat condition
    I2C1CONbits.SEN = 0;        // init I2C start condition
    
    // set baud rate
    I2C1BRG = 93;
    
    // enable the module
    I2C1CONbits.I2CEN = 1;
}