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
#include "odometry.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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

int16_t printOdometry(void) {
    char buffer[200];
    
    // TODO: print
    //extern volatile float velocity[3];       // x, y, z velocity (e.g., mm/s)
    //extern volatile float position[3];       // x, y, z position (e.g., mm)
    //extern volatile float yaw;               // Yaw angle in degrees or radians
    
    /*
    snprintf(buffer, sizeof(buffer),
         "Velocity: X=%.2f mm/s, Y=%.2f mm/s, Z=%.2f mm/s | Position: X=%.2f mm, Y=%.2f mm, Z=%.2f mm | Yaw: %.2f deg\r\n",
         velocity[0], velocity[1], velocity[2],
        position[0], position[1], position[2],
         yaw);
     */
    
    snprintf(buffer, sizeof(buffer),
         "Left: %d, Right: %d, Left: %.2f deg, Right: %.2f deg, Vel Left: %.2f dps, Vel Right: %.2f dps, Yaw: %.2f deg\r\n", 
            getPositionInCounts(ENCODER_LEFT), 
            getPositionInCounts(ENCODER_RIGHT), 
            getPositionInDeg(ENCODER_LEFT), 
            getPositionInDeg(ENCODER_RIGHT), 
            getVelocityInDegPerSecond(ENCODER_LEFT), 
            getVelocityInDegPerSecond(ENCODER_RIGHT), 
            getEncoderYawDeg()
            );
    putsUART1(buffer);

    //imuScaleAccelMeasurements(rawAccelMeasurements, accelMeasurements);
    
    //char measurementStr[70];
    //snprintf(measurementStr, 70, "Accelerometer [g]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accelMeasurements[0], accelMeasurements[1], accelMeasurements[2]);
    //putsUART1(measurementStr);

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
    
    initQEI(ENCODER_LEFT, 0);  // configure Quadrature Encoder 1
    initQEI(ENCODER_RIGHT, 0); // configure Quadrature Encoder 2
    
    initDmaChannel4(); // Initialize DMA to copy sensor readings in the background
    setupADC1();       // Initialize ADC to sample sensor reading
    startADC1();       // Start to sample sensor readings
    
    __delay_ms(100); // Wait a bit for the peripherals to start up
    
    oledSetup();    // Setup oled display
    
    imuSetup(GYRO_RANGE_500DPS, ACCEL_RANGE_2G, MAG_MODE_100HZ, TEMP_ON); // configure IMU over I2C
    imuCalibrateGyro(); // Calibrate gyroscope.
    imuCalibrateAccel(); // Calibrate accelerometer.
    
    initMotorsState();
    initMouseState();
    
    initSwitch1();     // Initialize switch 1 for interrupts
    registerSwitchCallback(SWITCH_1, toggleMotors);
    
    initTimerInMs(TIMER_1, 10); // main 10ms interrupt for high-level logic
    initTimerInMs(TIMER_2, 2);  // high frequency 2ms timer interrupt for sensor readings and rtttl

    //parseAllSongs();
    //playSong(SONG_MUPPETS, true, TIMER_2);
    
    initTimerInMs(TIMER_3, 300); // 100ms timer interrupt for testing
    
    //registerTimerCallback(TIMER_1, moveForward);
    //registerTimerCallback(TIMER_3, moveForward);

    setupOdometry(TIMER_2); // track odometry every 1ms
    registerTimerCallback(TIMER_3, printOdometry);

    
    //steerMotors(30, 30);
    //setMotorPower(MOTOR_LEFT, 50);
    //SET_MOTOR_LEFT(50);
    
    // setTimerState(TIMER_3, 1);
    
    // initTimerInMs(TIMER_32_COMBINED, 250); //creates a 10ms timer interrupt
    // setTimerState(TIMER_32_COMBINED, 1);

    //for (uint8_t i = 0; i < 6; i++) {
    //    int8_t degrees = i % 2 == 0 ? 90 : -90;
    //    turnDegrees(TIMER_1, degrees, 50);
    //}
    
    //moveDistance(TIMER_1, 200, 25);
    
    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;
    LED5 = LEDOFF;
}

void bootReset() {
    asm volatile ( "reset ");
}
