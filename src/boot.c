#include "configbits.h"
#include "IOconfig.h"
#include "clock.h"
#include "motorEncoders.h"
#include "pwm.h"
#include "timers.h"
#include "uart.h"
#include "i2c.h"
#include "imu.h"
#include "oled.h"
#include "adc.h"
#include "dma.h"
#include "rtttl.h"
#include "globalTimers.h"
#include "switches.h"

#include <stdint.h>

void bootSetup() {
    setupClock(); // configures oscillator circuit
    
    initGlobalTimekeeping(); // configures timekeeping since startup
    
    setupIO(); // configures inputs and outputs
    
    setupUART1(); // configures UART1
    setupPWM1(); // configure PWM1
    setupPWM2(); // configure PWM2
    setupI2C1(); // configure I2C
    
    initQEI1(0); // configure Quadrature Encoder 1
    initQEI2(0); // configure Quadrature Encoder 2

    imuSetup(GYRO_RANGE_500DPS, ACCEL_RANGE_2G, MAG_MODE_100HZ, TEMP_ON); // configure IMU over I2C
    imuCalibrateGyro(); // Calibrate gyroscope.
    
    oledSetup();
    oledClearDisplay();
    oledRefresh();
    
    initDmaChannel4(); // Initialize DMA to copy sensor readings in the background
    setupADC1();       // Initialize ADC to sample sensor reading
    startADC1();       // Start to sample sensor readings

    initSwitch1();     // Initialize switch 1 for interrupts
    
    //initTimerInMs(TIMER_1, 100); //creates a 10ms timer interrupt
    // setTimerState(TIMER_1, 1);

    // initTimerInMs(TIMER_2, 50); //creates a 10ms timer interrupt
    // setTimerState(TIMER_2, 1);

    //initTimerInMs(TIMER_3, 1); // creates a 1ms timer interrupt
    //parseAllSongs();
    //playSong(SONG_MUPPETS, true);
    
    // setTimerState(TIMER_3, 1);
    
    // initTimerInMs(TIMER_32_COMBINED, 250); //creates a 10ms timer interrupt
    // setTimerState(TIMER_32_COMBINED, 1);

    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;
    LED5 = LEDOFF;
}

void bootReset() {
    asm volatile ( "reset ");
}
