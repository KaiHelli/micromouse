#include <xc.h>

void setupClock() {
    /*** oscillator setup --------------------------------------------------
    * The external oscillator runs at 16MHz
    * PLL is used to generate 80 MHz clock (FOSC)
    * The relationship between oscillator and cycle frequency: FCY = FOSC/2
    * Have a look at "PLL Configuration" paragraph in the mcu manual

    * Result: FCY = 0.5 * (16MHz*40/(4*2)) = 40 MIPS, Tcycle=25nsec
   ---------------------------------------------------------------------***/
    CLKDIVbits.PLLPRE = 2; // N1 = input/4 (2+2)
    PLLFBDbits.PLLDIV = 38; // set PPL to M=40 (38+2)
    CLKDIVbits.PLLPOST = 0; // N2 = output/2 (0+2))
    
    /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(OSCCON || 0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b011)
        ;

    // In reality, give some time to the PLL to lock
    while (OSCCONbits.LOCK != 1)
        ; // Wait for PLL to lock
}