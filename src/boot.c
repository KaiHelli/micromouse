#include "configbits.h"
#include "IOconfig.h"
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
#include "motors.h"
#include "mouseController.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdint.h>
#include <stdbool.h>

int16_t toggleMotors(void) {
    static bool standby = 0;
    standby = !standby;
    setMotorsStandbyState(standby);
    return 1;
}

int16_t readIMU(void) {
    imuReadAccel();
    
    return 1;
}

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
    
    initDmaChannel4(); // Initialize DMA to copy sensor readings in the background
    setupADC1();       // Initialize ADC to sample sensor reading
    startADC1();       // Start to sample sensor readings
    
    __delay_ms(100); // Wait a bit for the peripherals to start up
    
    imuSetup(GYRO_RANGE_500DPS, ACCEL_RANGE_2G, MAG_MODE_100HZ, TEMP_ON); // configure IMU over I2C
    //imuCalibrateGyro(); // Calibrate gyroscope.
    imuCalibrateAccel(); // Calibrate accelerometer.
    
    oledSetup();    // Setup oled display
    
    initMotorsState();
    initMouseState();
    
    initSwitch1();     // Initialize switch 1 for interrupts
    registerSwitchCallback(SWITCH_1, toggleMotors);
    
    initTimerInMs(TIMER_1, 10); // main 10ms interrupt for high-level logic
    initTimerInMs(TIMER_2, 1);  // high frequency 1ms timer interrupt for sensor readings and rtttl

    //parseAllSongs();
    //playSong(SONG_MUPPETS, true, TIMER_2);
    
    initTimerInMs(TIMER_3, 300); // 100ms timer interrupt for testing
    
    registerTimerCallback(TIMER_1, moveForward);
    //registerTimerCallback(TIMER_3, moveForward);

    //steerMotors(30, 30);
    //setMotorPower(MOTOR_LEFT, 50);
    //SET_MOTOR_LEFT(50);
    
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
