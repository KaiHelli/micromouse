#ifndef IOCONFIG_H
#define IOCONFIG_H

#define SW1 !PORTCbits.RC7
#define IMU_INT PORTBbits.RB7

#define MA_DIR1 LATAbits.LATA7
#define MA_DIR2 LATAbits.LATA10
#define MB_DIR1 LATAbits.LATA0
#define MB_DIR2 LATAbits.LATA1

#define MA_PWM_MODULE  1
#define MA_PWM_CHANNEL 2

#define MB_PWM_MODULE  1
#define MB_PWM_CHANNEL 1

#define LED5_PWM_MODULE  1
#define LED5_PWM_CHANNEL 3

#define BUZZ_PWM_MODULE  2
#define BUZZ_PWM_CHANNEL 1

#define M_STDBY LATBbits.LATB15

#define LED1 LATBbits.LATB4
#define LED2 LATAbits.LATA4
#define LED3 LATAbits.LATA9
#define LED4 LATCbits.LATC3
#define LED5 LATBbits.LATB10

#define GPIO09 LATBbits.LATB11
#define GPIO11 LATBbits.LATB12
#define GPIO26 LATCbits.LATC1
#define GPIO27 LATCbits.LATC2
#define GPIO32 LATAbits.LATA8

#define LEDON 0
#define LEDOFF 1

void setupIO();

#endif /* IOCONFIG_H */