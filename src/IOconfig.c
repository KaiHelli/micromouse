#include "IOconfig.h"
#include "interrupts.h"
#include <xc.h>

void setupIO()
{

    int i;
    
    // set all pins to digital IO
    AD1PCFGL = 0xFFFF;
    
    // NOTE: adc setup is done in adc.h / adc.c
    // NOTE: pwm mapping is done in pwm.h / pwm.c

    // set motor directions as output
    TRISAbits.TRISA7 = 0;
    TRISAbits.TRISA10 = 0;
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    
    // set motor standby as output
    TRISBbits.TRISB15 = 0;
    
    // set LEDs as output
    TRISBbits.TRISB4 = 0;
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA9 = 0;
    TRISCbits.TRISC3 = 0;
    TRISBbits.TRISB10 = 0;
    
    // set GPIO PINS as output
    TRISBbits.TRISB11 = 0;
    TRISBbits.TRISB12 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISAbits.TRISA8 = 0;
    
    // set UART1 TX as output
    TRISCbits.TRISC5 = 0;
    
    // PIN MAPPING

    // before we map, we need to unlock
    __builtin_write_OSCCONL(OSCCON & 0xbf); // clear bit 6 (unlock, they are usually write protected)

    // BUTTON INTERRUPT MAPPING
    
    RPINR0bits.INT1R = 23;      // mapped RP23 as External Interrupt 1
    
    // UART MAPPING
    
    // PERIPHERAL receives data from which INPUT
    RPINR18bits.U1RXR = 20; // mapped RP20 as U1 RX
    // PERIPHERAL sends data to which OUTPUT
    RPOR10bits.RP21R = 0b00011; // mapped RP21 as U1 TX
    
    // QUADRATURE ENCODER MAPPING
    
    // PERIPHERAL QE1 Channel A, receives data from RP24
    RPINR14bits.QEA1R = 24;
    // PERIPHERAL QE1 Channel B, receives data from RP25
    RPINR14bits.QEB1R = 25;

    // PERIPHERAL QE2 Channel A, receives data from RP1
    RPINR16bits.QEA2R = 1;
    // PERIPHERAL QE2 Channel B, receives data from RP0
    RPINR16bits.QEB2R = 0;
    

    // after mapping we lock again
    __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS registers (lock again!)

    for (i = 0; i < 30000; i++)
        ; // short dirty delay for changes to take effect,
}
