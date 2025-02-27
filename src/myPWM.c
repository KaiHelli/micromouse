
#include "myPWM.h"
#include <xc.h>
void setupPWM()
{
    /* PWM1H1 *, configured to 1kHz, based on fcyc = 40.000 MIPS, Tcycle=25nsec
     * 1ms/25nsec = 40000 (fits in 15 bits)
     * of course, we could use a pre-scaler and end up somewhere else
     */
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    P1TCONbits.PTCKPS = 0b00; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    P1TPER = MYPWM_MAX / 2; // 15 bit register
    PWM1CON1bits.PMOD1 = 1; // set PWM unit 1 to independent mode

    PWM1CON1bits.PEN1H = 1; // enable  PWM driver PWM1H1
    PWM1CON1bits.PEN2H = 0; // disable PWM driver
    PWM1CON1bits.PEN3H = 0; // disable PWM driver
    PWM1CON1bits.PEN1L = 0; // disable PWM driver
    PWM1CON1bits.PEN2L = 0; // disable PWM driver
    PWM1CON1bits.PEN3L = 0; // disable PWM driver

    P1TCONbits.PTEN = 1; // Switch on PWM generator
    // P1DC1 = .1*MYPWM_MAX; //to get 100% DC, you need to write twice the PER Value (2*40000)

    // Leave channels disabled for now
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
}