/*
 * File:   main.c
 * Author: Alexander Lenz
 *
 * Created on 27 Nov 2020, 09:36
 */

/// Configuration Bits---------------------------

// FBS
#pragma config BWRP = WRPROTECT_OFF // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC // Start with Internal RC Oscillator
#pragma config IESO = OFF // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS1 // Watchdog Timer Postscaler (1:1)
#pragma config WDTPRE = PR128 // WDT Prescaler (1:128)
#pragma config WINDIS = OFF // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128 // POR Timer Value (128ms)
#pragma config ALTI2C = ON // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD3 // Comm Channel Select (Communicate on PGC3/EMUC3 and PGD3/EMUD3)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG is Disabled)

/// Include headers-------------------------------
#include "IOconfig.h"
#include "clock.h"
#include "motorEncoders.h"
#include "myPWM.h"
#include "myTimers.h"
#include "serialComms.h"
#include "i2c.h"
#include <stdint.h>
#include <xc.h>

/*
 *
 */
int16_t main()
{
    setupClock(); // configures oscillator circuit
    setupIO(); // configures inputs and outputs
    setupUART1(); // configures UART
    setupPWM(); // configure PWM
    setupI2C1(); // configure I2C
    
    initQEI1(0); // configure Quadrature Encoder 1
    initQEI2(0); // configure Quadrature Encoder 1

    // initTimerInMs(10, 1); //creates a 10ms timer interrupt
    // startTimer1();

    // initTimerInMs(50, 2); //creates a 10ms timer interrupt
    // startTimer2();

    initTimerInMs(10, 3); // creates a 10ms timer interrupt
    startTimer3();

    // initTimerInMs(250, 32); //creates a 10ms timer interrupt
    // startTimer32_combined();

    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;
    LED5 = LEDOFF;

    while (1) {
    };

    return 0;
}
