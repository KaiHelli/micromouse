#include <xc.h>
#include "myPWM.h"
#include "IOconfig.h"

void setupPWM1()
{
    /* PWM1, configured to 1kHz, based on fcyc = 40.000 MIPS, Tcycle=25nsec
     * 1ms/25nsec = 40000 (fits in 15 bits)
     * of course, we could use a pre-scaler and end up somewhere else
     */
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    P1TCONbits.PTCKPS = 0b01; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    P1TPER = PWM_1KHZ / 2; // 15 bit register
    PWM1CON1bits.PMOD1 = 1; // set PWM unit 1 to independent mode

    PWM1CON1bits.PEN1H = 1; // enable PWM driver PWM1H1
    PWM1CON1bits.PEN2H = 1; // enable PWM driver PWM1H2
    PWM1CON1bits.PEN3H = 1; // enable PWM driver PWM1H3
    PWM1CON1bits.PEN1L = 0; // disable PWM driver
    PWM1CON1bits.PEN2L = 0; // disable PWM driver
    PWM1CON1bits.PEN3L = 0; // disable PWM driver

    P1TCONbits.PTEN = 1; // Switch on PWM generator
    // P1DC1 = .1*PWM_1KHZ; //to get 100% DC, you need to write twice the PER Value (2*40000)

    // Leave channels disabled for now
    MA_DIR1 = 0;
    MA_DIR2 = 1;
    MB_DIR1 = 0;
    MB_DIR2 = 1;
    M_STDBY = 0;
    P1DC1 = 0.1 * PWM_MOTOR_MAX_DC * PWM_1KHZ;
    P1DC2 = 0.1 * PWM_MOTOR_MAX_DC * PWM_1KHZ;
    P1DC3 = 0;
}

void setupPWM2()
{
    /* PWM2, configured to 1kHz, based on fcyc = 40.000 MIPS, Tcycle=25nsec
     * 1ms/25nsec = 40000 (fits in 15 bits)
     * of course, we could use a pre-scaler and end up somewhere else
     */
    P2TCONbits.PTEN = 0; // Switch off PWM generator
    P2TCONbits.PTCKPS = 0b01; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    P2TPER = PWM_1KHZ / 2; // 15 bit register
    PWM2CON1bits.PMOD1 = 1; // set PWM unit 2 to independent mode

    PWM2CON1bits.PEN1H = 1; // enable PWM driver PWM1H1
    PWM2CON1bits.PEN1L = 0; // disable PWM driver

    P2TCONbits.PTEN = 1; // Switch on PWM generator
    // P2DC1 = 0.001*PWM_1KHZ; //to get 100% DC, you need to write twice the PER Value (2*40000)

    // Leave channels disabled for now
    P2DC1 = 0;
}


//void setPWMDC(float dc) {}

//void setFreq(int freq) {}