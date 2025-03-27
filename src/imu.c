#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include "i2c.h"
#include "imu.h"
#include "uart.h"
#include "IOconfig.h"

static uint8_t imuWhoAmI = 0xFF;  // Buffer to store read result
static uint8_t magWhoAmI = 0xFF;  // Buffer to store read result

// Arrays holding LSB-per-unit for each selectable range:
static const float gyroLSBTable[]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const uint16_t accelLSBTable[] = { 16384, 8192, 4096, 2048 };

// These will store the selected LSB values:
static float gyroLSB  = 131.0f;  // default -- gyroLSBTable[0]
static float accelLSB = 16384;   // default -- accelLSBTable[0]

volatile int16_t rawGyroMeasurements[3];
volatile int16_t rawAccelMeasurements[3];
volatile int16_t rawMagMeasurements[3];
volatile int16_t rawTempMeasurement;

// Local variables that are "unstable".
volatile int16_t localGyroMeasurements[3];
volatile int16_t localAccelMeasurements[3];
volatile int16_t localMagMeasurements[3];
volatile int16_t localTempMeasurement;

// Track the user bank we are currently on
static uint8_t currentBank = 0;

void imuReadGyroCb(bool success) {
    // Fix endianness
    for(int i = 0; i < 3; i++) {
        rawGyroMeasurements[i] = SWAP_BYTES(localGyroMeasurements[i]);
    }
    
    char measurementStr[70];
    
    float gyroMeasurements[3];
    
    imuScaleGyroMeasurements(rawGyroMeasurements, gyroMeasurements);
        
    snprintf(measurementStr, 70, "Gyroscope [dps]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", gyroMeasurements[0], gyroMeasurements[1], gyroMeasurements[2]);
    putsUART1(measurementStr);
}

void imuReadAccelCb(bool success) {
    // Fix endianness
    for(int i = 0; i < 3; i++) {
        rawAccelMeasurements[i] = SWAP_BYTES(localAccelMeasurements[i]);
    }
    
    char measurementStr[70];

    float accelMeasurements[3];
        
    imuScaleAccelMeasurements(rawAccelMeasurements, accelMeasurements);

    snprintf(measurementStr, 70, "Accelerometer [g]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accelMeasurements[0], accelMeasurements[1], accelMeasurements[2]);
    putsUART1(measurementStr);
}

void imuReadMagCb(bool success) {
    // Copy values
    for(int i = 0; i < 3; i++) {
        rawMagMeasurements[i] = localMagMeasurements[i];
    }
    
    char measurementStr[80];
    
    float magMeasurements[3];
    
    imuScaleMagMeasurements(rawMagMeasurements, magMeasurements);
    
    float heading =  magnetometerToHeading(magMeasurements);
    
    snprintf(measurementStr, 80, "Magnetometer [uT]: X = %1.2f\tY = %1.2f\tZ = %1.2f\tHeading = %1.2f deg\r\n", magMeasurements[0], magMeasurements[1], magMeasurements[2], heading);
    putsUART1(measurementStr);
}

void imuReadTempCb(bool success) {
    // Fix endianness
    rawTempMeasurement = SWAP_BYTES(localTempMeasurement);

    char measurementStr[70];

    float tempMeasurement;
    
    imuScaleTempMeasurements(&rawTempMeasurement, &tempMeasurement);

    snprintf(measurementStr, 70, "Temperature [C]: %1.2f\r\n", tempMeasurement);
    
    putsUART1(measurementStr);
}

void imuReadGyro(void) {
    static uint8_t measurementRegisterStart = ICM20948_GYRO_XOUT_H;
    
    // Switch to User Bank 0
    imuSetUsrBank(0);
    putsI2C1(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) localGyroMeasurements, 6, imuReadGyroCb);
}

bool imuReadGyroSync(int16_t rawGyroMeasurements[3], float scaledGyroMeasurements[3]) {
    static uint8_t measurementRegisterStart = ICM20948_GYRO_XOUT_H;
    
    bool status = 1;
    
    // Switch to User Bank 0
    imuSetUsrBank(0);
    
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) rawGyroMeasurements, 6);
    
    if (!status) {
        return status;
    }
    
    // Fix endianness
    for (int i = 0; i < 3; i++) {
        rawGyroMeasurements[i] = SWAP_BYTES(rawGyroMeasurements[i]);
    }
    
    if (scaledGyroMeasurements) {
        imuScaleGyroMeasurements(rawGyroMeasurements, scaledGyroMeasurements);
    }
    
    return status;
}

void imuReadAccel(void) {
    static uint8_t measurementRegisterStart = ICM20948_ACCEL_XOUT_H;
    
    // Switch to User Bank 0
    imuSetUsrBank(0);
    putsI2C1(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) localAccelMeasurements, 6, imuReadAccelCb);
}

void imuReadMag(void) {
    static uint8_t measurementRegisterStart = AK09916_XOUT_L;
    
    // Reading measurements locks measured data until register ST2 is read.
    putsI2C1(I2C_IMU_MAG_ADDR, &measurementRegisterStart, 1, (uint8_t*) localMagMeasurements, 6, NULL);
    
    // To unlock measurement registers after reading, status register ST2 has to be read. 
    static uint8_t measurementStatusRegister = AK09916_ST2;
    static uint8_t measurementStatus;
    
    putsI2C1(I2C_IMU_MAG_ADDR, &measurementStatusRegister, 1, &measurementStatus, 1, imuReadMagCb);
    
    // TODO: Check HOFL bit in measurementStatus for magnetic sensor overflow in callback and only update measurements if data is valid.
}


void imuReadTemp(void) {
    static uint8_t measurementRegisterStart = ICM20948_TEMP_OUT_H;
    
    // Switch to User Bank 0
    imuSetUsrBank(0);
    putsI2C1(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) &localTempMeasurement, 2, imuReadTempCb);
}

// -----------------------------------------------------------------------------
// This callback function is invoked from the I2C ISR after the transaction
// has completed (success or error).
// -----------------------------------------------------------------------------
static void imuReadWhoAmICb(bool success)
{
    // The whoAmI variable now holds the data read from the WHO_AM_I register.
    // According to the ICM-20948 datasheet, this should be 0xEA.
    char whoAmIStr[50];

    snprintf(whoAmIStr, 50, "ICM-20948 WHO_AM_I = 0x%02X\r\n", imuWhoAmI);

    putsUART1(whoAmIStr);
}

// -----------------------------------------------------------------------------
// Starts an asynchronous read of the WHO_AM_I register. We send a single-byte
// register address, then read back one byte into 'whoAmI'.
// -----------------------------------------------------------------------------
void imuReadWhoAmI(void)
{
    static uint8_t regAddr = ICM20948_WHO_AM_I;
    
    // Switch to User Bank 0
    imuSetUsrBank(0);
    putsI2C1(I2C_IMU_GYRO_ADDR, &regAddr, 1, &imuWhoAmI, 1, imuReadWhoAmICb);
}

// -----------------------------------------------------------------------------
// This callback function is invoked from the I2C ISR after the transaction
// has completed (success or error).
// -----------------------------------------------------------------------------
static void imuReadWhoAmIMagCb(bool success)
{
    // The whoAmI variable now holds the data read from the WHO_AM_I register.
    // According to the datasheet, this should be 0x09.
    char whoAmIStr[50];

    snprintf(whoAmIStr, 50, "AK09916 WHO_AM_I = 0x%02X\r\n", magWhoAmI);

    putsUART1(whoAmIStr);
}

// -----------------------------------------------------------------------------
// Starts an asynchronous read of the WHO_AM_I register. We send a single-byte
// register address, then read back one byte into 'whoAmI'.
// -----------------------------------------------------------------------------
void imuReadWhoAmIMag(void)
{
    static uint8_t regAddr = AK09916_WHO_AM_I;
    putsI2C1(I2C_IMU_MAG_ADDR, &regAddr, 1, &magWhoAmI, 1, imuReadWhoAmIMagCb);
}

void imuSelfTest(void)
{
    
}

void magSelfTest(void) 
{
    // Implementation placeholder
}

// Synchronously configure the IMU.
void imuSetup(GyroRange_t gyroRange, AccelRange_t accelRange, MagMode_t magMode, TempMode_t tempMode)
{
    bool status = 1; // track the status of all operations
    
    // USR0 / PWR_MGMT_1
    // bit 7 -> 1 | reset IMU
    // 0x41 on reset -> change to 0xC1
    static uint8_t imuRstData[] = { ICM20948_PWR_MGMT_1, 0xC1 };
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, imuRstData, 2, NULL, 0);
    
    // Wait for the device to reset.
    __delay_ms(5);
    
    // USR0 / PWR_MGMT_1
    // bit 6 -> 0 | wake from sleep mode
    // bit 3 -> 1 | enable/disable temperature sensor
    // 0x41 on reset -> change to 0x01 / 0x05
    
    static uint8_t pwrData[] = { ICM20948_PWR_MGMT_1, 0x01 };
    pwrData[1] |= tempMode << 3;
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, pwrData, 2, NULL, 0);

    // USR0 / INT_PIN_CFG
    // bit 1 -> 1 | bridge auxiliary I2C bus with main bus (for magnetometer)
    // 0x00 on reset -> change to 0x02
    static uint8_t i2cData[] = { ICM20948_INT_PIN_CFG, 0x02 };
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, i2cData, 2, NULL, 0);
    
    // Switch to User Bank 2
    imuSetUsrBank(2);
    
    // USR2 / ODR_ALIGN_EN
    // bit 0 -> 1 | align sampling rates of sensors
    // 0x00 on reset -> change to 0x01
    static uint8_t odrAlignData[] = { ICM20948_ODR_ALIGN_EN, 0x01 };
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, odrAlignData, 2, NULL, 0);
    
    // USR2 / GYRO_SMPLRT_DIV
    // bit 7:0 -> Gyro sample rate divider
    // 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
    // 0x00 on reset
    
    // NOTE: left at 1.1kHz for now
    
    // USR2 / GYRO_CONFIG_1
    // bit 5:3 -> Gyro low pass filter configuration (see table 16)
    // bit 2:1 -> Gyro dynamic range selection (250, 500, 1000, 2000 dps)
    // bit 0 -> enable / disable low pass filter
    
    // Set gyro range selection.
    uint8_t gyroConfig = ((uint8_t)gyroRange << 1 | 6 << 3);
    // Enable LPF in bit 0:
    gyroConfig |= 0x01;
    
    static uint8_t gyroData[] = { ICM20948_GYRO_CONFIG_1, 0x0 };
    gyroData[1] = gyroConfig;
    
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, gyroData, 2, NULL, 0);
    
    // Store the selected LSB
    gyroLSB = gyroLSBTable[gyroRange];
    
    // USR2 / GYRO_CONFIG_2
    // bit 5:3 -> X/Y/Z self-test enable
    // bit 2:0 -> averaging filter settings for low-power mode
    
    // USR2 / ACCEL_SMPLRT_DIV_1
    // bit 3:0 -> MSB for accelerometer sample rate division
    
    // USR2 / ACCEL_SMPLRT_DIV_2
    // bit 7:0 -> LSB for accelerometer sample rate division
    // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
    
    // NOTE: left at 1.125kHz for now
    
    // USR2 / ACCEL_CONFIG
    // bit 5:3 -> Accel low pass filter configuration (see table 18)
    // bit 2:1 -> Accel dynamic range selection (2g, 4g, 8g, 16g)
    // bit 0 -> enable / disable low pass filter
    
    // Set accelerometer range selection.
    uint8_t accelConfig = ((uint8_t)accelRange << 1);
    // Enable LPF in bit 0:
    accelConfig |= 0x01;
    
    static uint8_t accelData[] = { ICM20948_ACCEL_CONFIG, 0x0 };
    accelData[1] = accelConfig;
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, accelData, 2, NULL, 0);

    // Store the selected LSB
    accelLSB = accelLSBTable[accelRange];
    
    // USR2 / ACCEL_CONFIG_2
    // bit 4:2 -> X/Y/Z self-test enable
    // bit 1:0 -> averaging filter settings for low-power mode
    
    // leave as is for now
    
    // MAG / CNTL3
    // bit 0 -> reset magnetometer
    // 0x00 on reset -> change to 0x01
    static uint8_t magRstData[] = { AK09916_CNTL3, 0x01 };
    status &= putsI2C1Sync(I2C_IMU_MAG_ADDR, magRstData, 2, NULL, 0);
    
    // Wait for magnetometer to reset.
    __delay_ms(5);
    
    // MAG / CNTL2
    // bit 4:0 -> Magnetometer operation mode setting
    static uint8_t magData[] = { AK09916_CNTL2, 0x0 };
    magData[1] = magMode;
    status &= putsI2C1Sync(I2C_IMU_MAG_ADDR, magData, 2, NULL, 0);
    
    // Wait for settings to take effect.
    __delay_ms(1000);
    
    if (status) {
        putsUART1("IMU configured.\r\n");
    } else {
        putsUART1("Error setting up the IMU!\r\n");
    }
}

bool imuCalibrateGyro() {
    putsUART1("Calibrating IMU Gyroscope. Hold still.\r\n");
    
    const uint16_t numSamples = 500;
    uint16_t numMeasurements = 0;
    
    // Use float for accuracy of running average calculations
    float runningAverage[3] = {0.0f, 0.0f, 0.0f};

    while (numMeasurements < numSamples) {
        int16_t rawGyroMeasurements[3];

        bool status = imuReadGyroSync(rawGyroMeasurements, NULL);

        if (!status) {
            continue;
        }

        // Incremental running average calculation
        for (uint8_t axis = 0; axis < 3; axis++) {
            runningAverage[axis] = ((runningAverage[axis] * numMeasurements) + rawGyroMeasurements[axis])
                                    / (numMeasurements + 1);
        }

        numMeasurements++;

        // At a sampling rate of 1.1kHz, wait 1ms before next reading
        __delay_ms(10);
    }
    
    // After calibration, offsets are available in runningAverage[]
    // (Write these values to the IMU offset registers as needed)
    imuSetUsrBank(2);
    
    // Round float values to int16_t before writing offsets
    int16_t gyroOffsets[3];
    
    // The offsets have to be set in the +-1000 dps sensitivity range. Therefore,
    // we might have to scale it.
    float sensitivityScaling = gyroLSB / gyroLSBTable[GYRO_RANGE_1000DPS];
    
    for (int axis = 0; axis < 3; axis++) {
        gyroOffsets[axis] = (int16_t)roundf(-runningAverage[axis] / sensitivityScaling);
    }
    
    bool status = 1;
    
    uint8_t offsetValues[7] = {
        ICM20948_XG_OFFSET_H,             // Register start address
        HIGH_BYTE(gyroOffsets[0]), LOW_BYTE(gyroOffsets[0]),
        HIGH_BYTE(gyroOffsets[1]), LOW_BYTE(gyroOffsets[1]),
        HIGH_BYTE(gyroOffsets[2]), LOW_BYTE(gyroOffsets[2])
    };
    
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, offsetValues, 7, NULL, 0);
    
    if (status) {
        putsUART1("Gyroscope calibrated.\r\n");
    } else {
        putsUART1("Error calibrating the IMU!\r\n");
    }
    
    return status;
}

void imuSetUsrBank(uint8_t bank)
{
    // Check if we actually have to do something.
    if (currentBank == bank) {
        return;
    }
    
    // Prepare a lookup table for bank 0..3
    //  bits 5:4 hold the user bank ID.
    static uint8_t bankRegs[4][2] =
    {
        { ICM20948_REG_BANK_SEL, 0 << 4 }, // Bank 0
        { ICM20948_REG_BANK_SEL, 1 << 4 }, // Bank 1
        { ICM20948_REG_BANK_SEL, 2 << 4 }, // Bank 2
        { ICM20948_REG_BANK_SEL, 3 << 4 }  // Bank 3
    };
    
    // Perform the async I2C write using the correct buffer
    // (the data remains valid because `bankRegs` is static)
    putsI2C1(I2C_IMU_GYRO_ADDR, bankRegs[bank], 2, NULL, 0, NULL);
    
    currentBank = bank;
}

void imuScaleGyroMeasurements(int16_t rawGyro[3], float scaledGyro[3])
{
    scaledGyro[0]  = (float)rawGyro[0]  / gyroLSB;
    scaledGyro[1]  = (float)rawGyro[1]  / gyroLSB;
    scaledGyro[2]  = (float)rawGyro[2]  / gyroLSB;
}

void imuScaleAccelMeasurements(int16_t rawAccel[3], float scaledAccel[3]) 
{
    scaledAccel[0] = (float)rawAccel[0] / accelLSB;
    scaledAccel[1] = (float)rawAccel[1] / accelLSB;
    scaledAccel[2] = (float)rawAccel[2] / accelLSB;
}

void imuScaleMagMeasurements(int16_t rawMag[3], float scaledMag[3])
{
    scaledMag[0] = (float)rawMag[0] * AK09916_UT_PER_LSB;
    scaledMag[1] = (float)rawMag[1] * AK09916_UT_PER_LSB;
    scaledMag[2] = (float)rawMag[2] * AK09916_UT_PER_LSB;
}

void imuScaleTempMeasurements(int16_t *rawTemp, float *scaledTemp)
{
    *scaledTemp = ((float)*rawTemp / ICM20948_LSB_PER_C) + 21;
}

float magnetometerToHeading(float scaledMag[3]) {
    float headingRadians = atan2f(scaledMag[1], scaledMag[0]); // atan2(Y, X)
    float headingDegrees = headingRadians * (180.0f / M_PI);

    // Convert negative degrees (range -180 to +180) to compass range (0 to 360)
    if (headingDegrees < 0) {
        headingDegrees += 360.0f;
    }

    return headingDegrees;
}

float dpsToRadps(float dps)
{
    return dps * (M_PI / 180.0f);
}