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
#include "constants.h"
#include "move.h"
#include "sensors.h"
#include "mazeSolver.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>


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

int16_t printSensorReadings(void) {
    #define ROBOTDISTANCE
    #ifdef ROBOTDISTANCE
    uint32_t left  = getRobotDistanceUm(SENSOR_LEFT);
    uint32_t front = getRobotDistanceUm(SENSOR_CENTER);
    uint32_t right = getRobotDistanceUm(SENSOR_RIGHT);
    #else
    uint32_t left  = getSensorDistanceUm(SENSOR_LEFT);
    uint32_t front = getSensorDistanceUm(SENSOR_CENTER);
    uint32_t right = getSensorDistanceUm(SENSOR_RIGHT);
    #endif
    
    uint8_t buffer[20];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &left, sizeof(left));
    idx += sizeof(left);
    
    memcpy(&buffer[idx], &front, sizeof(front));
    idx += sizeof(front);
    
    memcpy(&buffer[idx], &right, sizeof(right));
    idx += sizeof(right);
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);
    
    return 1;
}

int16_t estimateEncoderAcceleration(void) {
    static float lastPosDegLeft = 0.0f;
    float posDegLeft = getEncoderPositionDeg(ENCODER_LEFT);
    
    float velEstimateDps = (posDegLeft - lastPosDegLeft) / 1e-3f;
    lastPosDegLeft = posDegLeft;
    
    uint8_t buffer[10];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &velEstimateDps, sizeof(velEstimateDps));
    idx += sizeof(velEstimateDps);
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);
    
    return 1;
}

int16_t printEncoderVelocities(void) {
    /*
    char buffer[100];
    snprintf(buffer, sizeof(buffer),
         "Left: %ld, Right: %ld, Left: %.2f deg, Right: %.2f deg, Vel Left: %.2f dps, Vel Right: %.2f dps, Vel C Left: %ld, Vel C Right: %ld, Lin Vel: %.2f mmps\r\n", 
            getEncoderPositionCounts(ENCODER_LEFT), 
            getEncoderPositionCounts(ENCODER_RIGHT), 
            getEncoderPositionDeg(ENCODER_LEFT), 
            getEncoderPositionDeg(ENCODER_RIGHT), 
            getEncoderVelocityDegPerSec(ENCODER_LEFT), 
            getEncoderVelocityDegPerSec(ENCODER_RIGHT),
            getEncoderVelocityCountsPerSample(ENCODER_LEFT),
            getEncoderVelocityCountsPerSample(ENCODER_RIGHT),
            getEncoderYawRateRadPerSec(),
            getEncoderLinearVelocityMmPerSec()
            );
    putsUART1Str(buffer);
    */
    float velLeft = getEncoderVelocityDegPerSec(ENCODER_LEFT);
    float velRight = getEncoderVelocityDegPerSec(ENCODER_RIGHT);
    
    uint8_t buffer[15];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &velLeft, sizeof(velLeft));
    idx += sizeof(velLeft);
    
    memcpy(&buffer[idx], &velRight, sizeof(velRight));
    idx += sizeof(velRight);
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);

    return 1;
}

// visualize_imu_raw.dvws / imu_scaled.dvws
int16_t printIMU(bool fifo, bool scaled) {
    // Buffer must fit either:
    //   raw: 1 + 3*2 + 3*2 + 3*2 + 2 + 1 = 24 bytes
    //   scaled: 1 + 3*4 + 3*4 + 3*4 + 4 + 1 = 42 bytes
    uint8_t buffer[45];
    size_t idx = 0;

    // Temp storage for scaled values
    float scaledGyro[3], scaledAccel[3], scaledMag[3], scaledTemp;

    // If scaling requested, do all the calibration + scaling steps
    if (fifo && scaled) {
        imuScaleGyroMeasurementsFloat(rawFifoGyroMeasurements, scaledGyro);

        imuCalibrateAccelMeasurementsFloat(rawFifoAccelMeasurements, scaledAccel);
        imuScaleAccelMeasurementsFloat(scaledAccel, scaledAccel);

        imuCalibrateMagMeasurementsFloat(rawFifoMagMeasurements, scaledMag);
        imuScaleMagMeasurementsFloat(scaledMag, scaledMag);

        imuScaleTempMeasurementsFloat(&rawFifoTempMeasurement, &scaledTemp);
    }
    
    if (!fifo && scaled) {
        imuScaleGyroMeasurements(rawGyroMeasurements, scaledGyro);

        imuCalibrateAccelMeasurements(rawAccelMeasurements, scaledAccel);
        imuScaleAccelMeasurementsFloat(scaledAccel, scaledAccel);

        imuCalibrateMagMeasurements(rawMagMeasurements, scaledMag);
        imuScaleMagMeasurementsFloat(scaledMag, scaledMag);

        imuScaleTempMeasurements(&rawTempMeasurement, &scaledTemp);
    }

    // Start marker
    buffer[idx++] = FRAME_START_BYTE;

    // Select element size and byte?pointer to the right data
    const size_t elemSize = (scaled || fifo) ? sizeof(float) : sizeof(int16_t);
    const uint8_t *gyroBytes  = (const uint8_t *)(scaled ? (void*)scaledGyro  : (fifo ? (void*)rawFifoGyroMeasurements : (void*)rawGyroMeasurements));
    const uint8_t *accelBytes = (const uint8_t *)(scaled ? (void*)scaledAccel : (fifo ? (void*)rawFifoAccelMeasurements : (void*)rawAccelMeasurements));
    const uint8_t *magBytes   = (const uint8_t *)(scaled ? (void*)scaledMag   : (fifo ? (void*)rawFifoMagMeasurements : (void*)rawMagMeasurements));
    const uint8_t *tempBytes  = (const uint8_t *)(scaled ? (void*)&scaledTemp  : (fifo ? (void*)&rawFifoTempMeasurement : (void*)&rawTempMeasurement));

    // Copy 3 axes of gyro, accel, mag
    for (uint8_t axis = 0; axis < 3; axis++) {
        memcpy(&buffer[idx], gyroBytes  + axis * elemSize, elemSize);
        idx += elemSize;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        memcpy(&buffer[idx], accelBytes + axis * elemSize, elemSize);
        idx += elemSize;
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
        memcpy(&buffer[idx], magBytes   + axis * elemSize, elemSize);
        idx += elemSize;
    }

    // Copy single temperature value
    memcpy(&buffer[idx], tempBytes, elemSize);
    idx += elemSize;

    // End marker
    buffer[idx++] = FRAME_END_BYTE;

    // Send it all out
    putsUART1(buffer, idx);

    return 1;
}

int16_t printIMU_trampoline() {
    return printIMU(true, true);
}

// visualize_odometry.dvws
int16_t printOdometry(void) {
    /*
    snprintf(buffer, sizeof(buffer),
         "Velocity: X=%.2f mm/s, Y=%.2f mm/s, Z=%.2f mm/s | Position: X=%.2f mm, Y=%.2f mm, Z=%.2f mm | Pitch: %.2f deg, Roll: %.2f deg, Yaw: %.2f deg\r\n",
         mouseVelocity[X], mouseVelocity[Y], mouseVelocity[Z],
         mousePosition[X], mousePosition[Y], mousePosition[Z],
         mouseAngle[PITCH]*RAD2DEG, mouseAngle[ROLL]*RAD2DEG, mouseAngle[YAW]*RAD2DEG);
    */
    
    uint8_t buffer[55];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        float angleDeg = mouseAngle[axis] * RAD2DEG;
        memcpy(&buffer[idx], &angleDeg, sizeof(mouseAngle[axis]));
        idx += sizeof(mouseAngle[axis]);
    }
    
    float accellPitchDeg = mouseAccelPitch * RAD2DEG;
    memcpy(&buffer[idx], &accellPitchDeg, sizeof(accellPitchDeg));
    idx += sizeof(accellPitchDeg);
    
    float accellRollDeg = mouseAccelRoll * RAD2DEG;
    memcpy(&buffer[idx], &accellRollDeg, sizeof(accellRollDeg));
    idx += sizeof(accellRollDeg);
    
    float magYawDeg = mouseMagYaw * RAD2DEG;
    memcpy(&buffer[idx], &magYawDeg, sizeof(mouseMagYaw));
    idx += sizeof(magYawDeg);
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        memcpy(&buffer[idx], &mouseVelocity[axis], sizeof(mouseVelocity[axis]));
        idx += sizeof(mouseVelocity[axis]);
    }
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        memcpy(&buffer[idx], &mousePosition[axis], sizeof(mousePosition[axis]));
        idx += sizeof(mousePosition[axis]);
    }
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);

    return 1;
}


int16_t startMaze(void) {
    setMotorsStandbyState(false);
    
    LED3 = LEDON;
    
    /*
    turnOrientation(TIMER_1, UP, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, LEFT, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, DOWN, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, RIGHT, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, UP, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, UP, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, RIGHT, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, DOWN, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, LEFT, 90, 2, getTimerFrequency(TIMER_1));
    turnOrientation(TIMER_1, UP, 90, 2, getTimerFrequency(TIMER_1));
    */
    
    //solveMaze();
    
    LED4 = LEDON;
    
    return 0;
}

int16_t initMouse(void) {
    uprintf("Initializing mouse state.\r\n");
    
    //centerMouseInCell();
    
    LED1 = LEDON;
    
    __delay_ms(2000); // User delay to press the reset and calibrate only when hands are off
    
    // imuCalibrateAccel(); // Calibrate accelerometer.
    imuCalibrateGyro();  // Calibrate gyroscope.
    
    __delay_ms(250);
    
    //setupOdometry(TIMER_1, 1); // track odometry
    registerTimerCallback(TIMER_1, triggerOdometryUpdate, 1);
    registerTimerCallback(TIMER_1, updateEncoderVelocities, 1);
    
    __delay_ms(2000);
    
    setMotorsStandbyState(false);
    
    resetControlAll();
    enableMouseControl();
    disableWallsControl();
    //setMaxForce(0.2f);
    initMouseController(TIMER_1, 1, getTimerFrequency(TIMER_1));
    
    sideSensorsCloseControl(true);
    //setIdealAngularSpeed(2.0);
    setTargetLinearSpeed(0.2);
    __delay_ms(3000);
    setIdealAngularSpeed(0.0);
    setTargetLinearSpeed(0.0);
    
    // Data Visualizer Callbacks
    //registerTimerCallback(TIMER_3, printEncoderVelocities, 1);
    //registerTimerCallback(TIMER_3, printOdometry, 1);
    //registerTimerCallback(TIMER_3, printIMU_trampoline, 1);
    //registerTimerCallback(TIMER_3, printSensorReadings, 1);
    
    registerSwitchCallback(SWITCH_1, startMaze);
    
    LED2 = LEDON;
    
    return 0;
}

void bootSetup() {
    setupClock(); // configures oscillator circuit
    
    initGlobalTimekeeping(); // configures timekeeping since startup
    
    setupIO(); // configures inputs and outputs
    
    setupUART1(); // configures UART1
    setupPWM1(); // configure PWM1
    setupPWM2(); // configure PWM2
    setupI2C1(); // configure I2C
    
    initMotorEncoders();
    
    initDmaChannel4(); // Initialize DMA to copy sensor readings in the background
    setupADC1();       // Initialize ADC to sample sensor reading
    startADC1();       // Start to sample sensor readings
    
    __delay_ms(100); // Wait a bit for the peripherals to start up
    
    oledSetup();    // Setup oled display
    
    FifoConfig_t fifoCfg = {
        .gyro   = true,
        .accel  = false,
        .mag    = false,
        .temp   = false
    };
    imuSetup(GYRO_RANGE_1000DPS, ACCEL_RANGE_2G, MAG_MODE_100HZ, TEMP_OFF, fifoCfg); // configure IMU over I2C
    
    //imuGetAccelCalibrationData();   // Output calibration data of accelerometer
    //imuGetMagCalibrationData();     // Output calibration data of magnetometer
    // For calibration -> remember to set current calibration to identity matrix and 0 bias beforehand)
    
    initTimerInUs(TIMER_1, 5333); // high frequency timer
    //initTimerInMs(TIMER_2, 5); 
    initTimerInMs(TIMER_3, 25); // 100ms timer interrupt for testing

    initSwitch1();     // Initialize switch 1 for interrupts
    
    //parseAllSongs();
    //playSong(SONG_MUPPETS, true, TIMER_2);
   
    __delay_ms(1000);
        
    //registerSwitchCallback(SWITCH_1, initMouse);
    initMouse();
    __delay_ms(1000);
    startMaze();

    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;
    LED5 = LEDOFF;
    
    while (1) {}
}

void bootReset() {
    asm volatile ( "reset ");
}
