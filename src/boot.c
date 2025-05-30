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

int16_t printEncoderValues(void) {
    uint8_t buffer[25];
    size_t idx = 0;
    
    int32_t leftDistTicks = getEncoderPositionCounts(ENCODER_LEFT);
    int32_t rightDistTicks = getEncoderPositionCounts(ENCODER_RIGHT);
    
    float leftDistUm = leftDistTicks * ENC_DIST_PER_TICK_UM;
    float rightDistUm = rightDistTicks * ENC_DIST_PER_TICK_UM;
    
    float distAvg = getEncoderAverageDistanceUm();
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &leftDistTicks, sizeof(leftDistTicks));
    idx += sizeof(leftDistTicks);
    
    memcpy(&buffer[idx], &rightDistTicks, sizeof(rightDistTicks));
    idx += sizeof(rightDistTicks);
    
    memcpy(&buffer[idx], &leftDistUm, sizeof(leftDistUm));
    idx += sizeof(leftDistUm);
    
    memcpy(&buffer[idx], &rightDistUm, sizeof(rightDistUm));
    idx += sizeof(rightDistUm);
    
    memcpy(&buffer[idx], &distAvg, sizeof(distAvg));
    idx += sizeof(distAvg);
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);
    
    return 1;
}

int16_t printMouseRotationalVelocity(void) {
    imuReadFifoSync();
    
    float scaledGyro;
    imuScaleGyroMeasurementFloat(&rawFifoGyroMeasurements[YAW], &scaledGyro);
    
    uint8_t buffer[25];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &scaledGyro, sizeof(scaledGyro));
    idx += sizeof(scaledGyro);
    memcpy(&buffer[idx], &powerPctLeft, sizeof(powerPctLeft));
    idx += sizeof(powerPctLeft);
    memcpy(&buffer[idx], &powerPctRight, sizeof(powerPctRight));
    idx += sizeof(powerPctRight);
    
    buffer[idx++] = FRAME_END_BYTE;
    
    putsUART1(buffer, idx);

    return 1;
}

int16_t printMouseVelocities(void) {
    float velLeft = getEncoderVelocityMmPerSec(ENCODER_LEFT);
    float velRight = getEncoderVelocityMmPerSec(ENCODER_RIGHT);
    float velMouse = getEncoderLinearVelocityMmPerSec();
    
    uint8_t buffer[25];
    size_t idx = 0;
    
    buffer[idx++] = FRAME_START_BYTE;
    
    memcpy(&buffer[idx], &velLeft, sizeof(velLeft));
    idx += sizeof(velLeft);
    memcpy(&buffer[idx], &velRight, sizeof(velRight));
    idx += sizeof(velRight);
    memcpy(&buffer[idx], &velMouse, sizeof(velMouse));
    idx += sizeof(velMouse);
    memcpy(&buffer[idx], &powerPctLeft, sizeof(powerPctLeft));
    idx += sizeof(powerPctLeft);
    memcpy(&buffer[idx], &powerPctRight, sizeof(powerPctRight));
    idx += sizeof(powerPctRight);
    
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

    // Select element size and byte-pointer to the right data
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
    
    solveMaze();
    
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
    //registerTimerCallback(TIMER_1, triggerOdometryUpdate, 1);
    //registerTimerCallback(TIMER_1, updateEncoderVelocities, 1);
    
    __delay_ms(2000);
    
    setMotorsStandbyState(false);
    
    resetControlAll();
    
    setMaxForce(0.5f);
    setMaxLinearSpeed(0.4f);

    initMouseController(TIMER_1, 1);
    enableMouseControl();
    //sideSensorsCloseControl(true);
    
    calibrateStartPosition();
    
    //pivot180(getMaxForce());
    
    //resetControlErrors();       // zero all PID integrators, etc.
    //disableWallsControl();      // start clean, no wall-following
    //setStartingPosition();      // record current encoder as cell start
    
    //targetStraightEncoders(getEncoderAverageDistanceUm(), CELL_DIMENSION_UM + SHIFT_DISTANCE_UM, 0.0);
    
    //moveForwardCenterCells(1, getMaxLinearSpeed(), 0.0f);
    //__delay_ms(250);
    //inplaceTurn( M_PI, 0.5f);
    //__delay_ms(250);
    //inplaceTurn( -M_PI, 0.5f);
    //__delay_ms(250);
    //targetStraight(0.18 * MICROMETERS_PER_METER, -0.18 * MICROMETERS_PER_METER, 0.0);
    
    /*
    
    float startMicrometers = 0.0f;
    float distance = 0.1 * MICROMETERS_PER_METER;
    
    //targetStraight(startMicrometers, distance, 0.0);
    
    
    for (uint8_t i = 0; i < 4 * 5; i++) {
        targetStraight(startMicrometers, distance, 0.0);
        inplaceTurn( M_PI / 2., 1.0f);
        
        startMicrometers += distance;
    }
     * */
    
    
    //sideSensorsCloseControl(true);
    //setIdealAngularSpeed(2.0);
    //setTargetLinearSpeed(0.2);
    //__delay_ms(3000);
    //setIdealAngularSpeed(0.0);
    //setTargetLinearSpeed(0.0);
    
    //registerTimerCallback(TIMER_2, updateEncoderVelocities, 1);
    
    // Data Visualizer Callbacks
    //registerTimerCallback(TIMER_2, printMouseRotationalVelocity, 1);
    //registerTimerCallback(TIMER_1, printMouseVelocities, 1);
    //registerTimerCallback(TIMER_3, printOdometry, 1);
    //registerTimerCallback(TIMER_3, printIMU_trampoline, 1);
    //registerTimerCallback(TIMER_3, printSensorReadings, 1);
    //registerTimerCallback(TIMER_3, printEncoderValues, 1);
    
    /*
    int8_t dutyCycles[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
    
    for (uint8_t i = 0; i < sizeof(dutyCycles); i++) {
        int8_t dir = i % 2 == 0 ? -1 : 1;
        
        setMotorPower(MOTOR_LEFT, dir * dutyCycles[i]);
        setMotorPower(MOTOR_RIGHT, dir * -dutyCycles[i]);
        
        __delay_ms(1000);
        
        setMotorPower(MOTOR_LEFT, 0);
        setMotorPower(MOTOR_RIGHT, 0);
        
        __delay_ms(1000);
    }
    */
    
    
    //squareUpByWiggle(10.0, 10, getMaxForce());
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
        .gyro   = false,
        .accel  = false,
        .mag    = false,
        .temp   = false
    };
    imuSetup(GYRO_RANGE_1000DPS, ACCEL_RANGE_2G, MAG_MODE_100HZ, TEMP_OFF, fifoCfg); // configure IMU over I2C
    
    //imuGetAccelCalibrationData();   // Output calibration data of accelerometer
    //imuGetMagCalibrationData();     // Output calibration data of magnetometer
    // For calibration -> remember to set current calibration to identity matrix and 0 bias beforehand)
    
    initTimerInUs(TIMER_1, 5333); // control timer
    initTimerInMs(TIMER_2, 1);    // buzzer timer
    initTimerInMs(TIMER_3, 25);   // debug timer

    initSwitch1();     // Initialize switch 1 for interrupts
    
    parseAllSongs();
   
    __delay_ms(1000);
        
    registerSwitchCallback(SWITCH_1, initMouse);

    //initMouse();
    
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
