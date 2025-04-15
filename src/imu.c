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
#include "odometry.h"

static uint8_t imuWhoAmI = 0xFF;  // Buffer to store read result
static uint8_t magWhoAmI = 0xFF;  // Buffer to store read result

// Arrays holding LSB-per-unit for each selectable range:
static const float gyroLSBTable[]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const uint16_t accelLSBTable[] = { 16384, 8192, 4096, 2048 };

// These will store the selected LSB values:
static float gyroLSB  = 131.0f;  // default -- gyroLSBTable[0]
static uint16_t accelLSB = 16384;   // default -- accelLSBTable[0]

volatile int16_t rawGyroMeasurements[3];
volatile int16_t rawAccelMeasurements[3];
volatile int16_t rawMagMeasurements[3];
volatile int16_t rawTempMeasurement;

volatile float gyroMeasurements[3];
volatile float accelMeasurements[3];
volatile float magMeasurements[3];
volatile float tempMeasurement;

// Local variables that are "unstable".
volatile int16_t localGyroMeasurements[3];
volatile int16_t localAccelMeasurements[3];
volatile int16_t localMagMeasurements[3];
volatile int16_t localTempMeasurement;

// Track the user bank we are currently on
static uint8_t currentBank = 0;

/**
 * @brief Callback for asynchronous gyroscope read completion.
 *
 * This function:
 * 1) Fixes raw big-endian sensor data into little-endian.
 * 2) Remaps the IMU?s physical gyro axes to the robot?s internal axes.
 *    - X rotation: Sensor: +X CW around Z axis -> Robot: +X CW around X axis
 *    - Y rotation: Sensor: +Y CCW around Y axis -> Robot: +Y CW around Y axis
 *    - Z rotation: Sensor: +Z CCW around Z axis -> Robot: +Z CW around C axis
 * 3) Stores the remapped gyro readings into global rawGyroMeasurements[].
 * 4) Invokes odometryIMUGyroUpdate() to integrate the new rotational data.
 */
void imuReadGyroCb(bool success) {
    if (!success) {
        putsUART1("Asynchronous IMU gyroscope error!\r\n");
        return;
    }
    
    // Fix endianness
    for(uint8_t axis = 0; axis < 3; axis++) {
        localGyroMeasurements[axis] = SWAP_BYTES(localGyroMeasurements[axis]);
    }
    
    // Remap from the IMU's physical frame to your robot frame
    int16_t remapped[3];
    remapped[0] =  localGyroMeasurements[0]; // +X should be CW from right
    remapped[1] = -localGyroMeasurements[1]; // +Y should be CW from back
    remapped[2] = -localGyroMeasurements[2]; // +Z should be CW from top
    
    // Copy the remapped data into the global rawAccelMeasurements
    for (uint8_t axis = 0; axis < 3; axis++) {
        rawGyroMeasurements[axis] = remapped[axis];
    }
    
    // Scale measurements
    // imuScaleGyroMeasurements(rawGyroMeasurements, gyroMeasurements);
    
    // Update odometry
    odometryIMUGyroUpdate();
    
    
    // char measurementStr[70];
    // snprintf(measurementStr, 70, "Gyroscope [dps]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", gyroMeasurements[0], gyroMeasurements[1], gyroMeasurements[2]);
    // putsUART1(measurementStr);
}


/**
 * @brief Callback for asynchronous accelerometer read completion.
 *
 * This function:
 * 1) Fixes raw big-endian sensor data into little-endian.
 * 2) Remaps the IMU?s physical axes to the robot?s internal axes:
 *    - IMU +X = left  -> Robot +X = right  (flip sign)
 *    - IMU +Y = back  -> Robot +Y = forward (flip sign)
 *    - IMU +Z = up    -> Robot +Z = up     (no flip)
 * 3) Stores the remapped readings into global rawAccelMeasurements[].
 * 4) Invokes odometryIMUAccelUpdate() to integrate these new data.
 */
void imuReadAccelCb(bool success) {
    if (!success) {
        putsUART1("Asynchronous IMU accelerometer error!\r\n");
        return;
    }
    
    // Fix endianness
    for(uint8_t axis = 0; axis < 3; axis++) {
        localAccelMeasurements[axis] = SWAP_BYTES(localAccelMeasurements[axis]);
    }
    
    // Remap from the IMU's physical frame to your robot frame
    int16_t remapped[3];
    remapped[0] = -localAccelMeasurements[0]; // X: left -> right
    remapped[1] = -localAccelMeasurements[1]; // Y: backward -> forward
    remapped[2] =  localAccelMeasurements[2]; // Z: up -> up
    
    // Copy the remapped data into the global rawAccelMeasurements
    for (uint8_t axis = 0; axis < 3; axis++) {
        rawAccelMeasurements[axis] = remapped[axis];
    }
    
    // Scale measurements
    // imuScaleAccelMeasurements(rawAccelMeasurements, accelMeasurements);
    
    // Update odometry
    odometryIMUAccelUpdate();
    
    //char measurementStr[70];
    //snprintf(measurementStr, 70, "Accelerometer [g]: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accelMeasurements[0], accelMeasurements[1], accelMeasurements[2]);
    //putsUART1(measurementStr);
}

/**
 * @brief Callback for asynchronous magnetometer read completion.
 *
 * This function:
 * 1) Copies the raw magnetometer data from localMagMeasurements[] to rawMagMeasurements[].
 * 2) Remaps the IMU?s physical magnetometer axes to the robot?s internal axes:
 *    - IMU +X = left   -> Robot +X = right   (flip sign)
 *    - IMU +Y = forward-> Robot +Y = forward (no flip)
 *    - IMU +Z = down   -> Robot +Z = up      (flip sign)
 * 3) Optionally calls scaling or heading computations.
 */
void imuReadMagCb(bool success) {
    if (!success) {
        putsUART1("Asynchronous IMU magnetometer error!\r\n");
        return;
    }
    
    // Copy values
    for(uint8_t axis = 0; axis < 3; axis++) {
        localMagMeasurements[axis] = localMagMeasurements[axis];
    }
    
    // Remap from the IMU's physical frame to your robot frame
    int16_t remapped[3];
    remapped[0] = -localMagMeasurements[0]; // X: left -> right
    remapped[1] =  localMagMeasurements[1]; // Y: forward -> forward
    remapped[2] = -localMagMeasurements[2]; // Z: down -> up
    
    // Copy the remapped data into the global rawAccelMeasurements
    for (uint8_t axis = 0; axis < 3; axis++) {
        rawMagMeasurements[axis] = remapped[axis];
    }
    
    // Scale measurements
    // imuScaleMagMeasurements(rawMagMeasurements, magMeasurements);
    
    // Calculate heading
    //float heading =  magnetometerToHeading(magMeasurements);
    
    //char measurementStr[80];
    //snprintf(measurementStr, 80, "Magnetometer [uT]: X = %1.2f\tY = %1.2f\tZ = %1.2f\tHeading = %1.2f deg\r\n", magMeasurements[0], magMeasurements[1], magMeasurements[2], heading);
    //putsUART1(measurementStr);
}

void imuReadTempCb(bool success) {
    if (!success) {
        putsUART1("Asynchronous IMU temperature error!\r\n");
        return;
    }
    
    // Fix endianness
    rawTempMeasurement = SWAP_BYTES(localTempMeasurement);

    // Scale measurement
    // imuScaleTempMeasurements(&rawTempMeasurement, &tempMeasurement);
    
    //char measurementStr[70];
    //snprintf(measurementStr, 70, "Temperature [C]: %1.2f\r\n", tempMeasurement);
    //putsUART1(measurementStr);
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
    for (uint8_t axis = 0; axis < 3; axis++) {
        rawGyroMeasurements[axis] = SWAP_BYTES(rawGyroMeasurements[axis]);
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

bool imuReadAccelSync(int16_t rawAccelMeasurements[3], float scaledAccelMeasurements[3]) {
    static uint8_t measurementRegisterStart = ICM20948_ACCEL_XOUT_H;
    
    bool status = 1;

    imuSetUsrBank(0);
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) rawAccelMeasurements, 6);
    
    if (!status) {
        return status;
    }

    // Fix endianness
    for (uint8_t axis = 0; axis < 3; axis++) {
        rawAccelMeasurements[axis] = SWAP_BYTES(rawAccelMeasurements[axis]);
    }

    if (scaledAccelMeasurements) {
        imuScaleAccelMeasurements(rawAccelMeasurements, scaledAccelMeasurements);
    }

    return status;
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
    uint8_t accelConfig = ((uint8_t)accelRange << 1 | 6 << 3);
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
    __delay_ms(100);
    
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

        bool readStatus = imuReadGyroSync(rawGyroMeasurements, NULL);

        if (!readStatus) {
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
    
    for (uint8_t axis = 0; axis < 3; axis++) {
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

bool imuCalibrateAccel() {
    putsUART1("Calibrating IMU Accelerometer. Hold still.\r\n");

    // Read factory calibration
    // Seems to be: X = 1190 Y = -1365 Z = -269
    int16_t accelOffsets[3] = {0, 0, 0};
    
    // Unfortunately addresses of registers are not consecutive.
    static uint8_t xOffsetRegisterStart = ICM20948_XA_OFFSET_H;
    static uint8_t yOffsetRegisterStart = ICM20948_YA_OFFSET_H;
    static uint8_t zOffsetRegisterStart = ICM20948_ZA_OFFSET_H;
    
    bool status = 1;
    
    // Switch to User Bank 1
    imuSetUsrBank(1);
    
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, &xOffsetRegisterStart, 1, (uint8_t*) &accelOffsets[0], 2);
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, &yOffsetRegisterStart, 1, (uint8_t*) &accelOffsets[1], 2);
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, &zOffsetRegisterStart, 1, (uint8_t*) &accelOffsets[2], 2);
    
    uint8_t temperatureBit[3] = {0, 0, 0};

    // Fix endianness
    for(uint8_t axis = 0; axis < 3; axis++) {
        accelOffsets[axis] = SWAP_BYTES(accelOffsets[axis]);
        
        // Store the lsb reserved bit of the low byte.
        temperatureBit[axis] = accelOffsets[axis] & 0x01;
        
        // Strip off the reserved bit from the bytes read.
        accelOffsets[axis] >>= 1;
    }
    
    const uint16_t numSamples = 500;
    uint16_t numMeasurements = 0;
    
    // Use float for accuracy of running average calculations
    float runningAverage[3] = {0.0f, 0.0f, 0.0f};

    while (numMeasurements < numSamples) {
        int16_t rawAccelMeasurements[3];

        bool readStatus = imuReadAccelSync(rawAccelMeasurements, NULL);

        if (!readStatus) {
            continue;
        }

        // Incremental running average calculation
        for (uint8_t axis = 0; axis < 3; axis++) {
            runningAverage[axis] = ((runningAverage[axis] * numMeasurements) + rawAccelMeasurements[axis])
                                    / (numMeasurements + 1);
        }

        numMeasurements++;

        // At a sampling rate of 1.125kHz, wait 1ms before next reading
        __delay_ms(10);
    }
    
    // We expect gravity to have 1G.
    runningAverage[2] -= (float) accelLSB;
    
    // After calibration, offsets are available in runningAverage[]
    // (Write these values to the IMU offset registers as needed)
    imuSetUsrBank(1);
    
    // The offsets have to be set in the +-16 G sensitivity range. Therefore,
    // we might have to scale it.
    float sensitivityScaling = accelLSB / accelLSBTable[ACCEL_RANGE_16G];
    
    // TODO: Magic factor of 2, that we are currently unsure where it stems from.
    sensitivityScaling *= 2;
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        accelOffsets[axis] += (int16_t)roundf(-runningAverage[axis] / sensitivityScaling);
        
        // Add temperature bit back in
        accelOffsets[axis] <<= 1;
        accelOffsets[axis] |= temperatureBit[axis];
    }
    
    uint8_t xOffsetValues[3] = {
        xOffsetRegisterStart, HIGH_BYTE(accelOffsets[0]), LOW_BYTE(accelOffsets[0])
    };
    uint8_t yOffsetValues[3] = {
        yOffsetRegisterStart, HIGH_BYTE(accelOffsets[1]), LOW_BYTE(accelOffsets[1])
    };
    uint8_t zOffsetValues[3] = {
        zOffsetRegisterStart, HIGH_BYTE(accelOffsets[2]), LOW_BYTE(accelOffsets[2])
    };
    
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, xOffsetValues, 3, NULL, 0);
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, yOffsetValues, 3, NULL, 0);
    status &= putsI2C1Sync(I2C_IMU_GYRO_ADDR, zOffsetValues, 3, NULL, 0);
    
    if (status) {
        putsUART1("Accelerometer calibrated.\r\n");
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
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleGyroMeasurement(&rawGyro[axis], &scaledGyro[axis]);
    }
}

void imuScaleGyroMeasurement(int16_t *rawGyro, float *scaledGyro)
{
    *scaledGyro = (float)(*rawGyro) / gyroLSB;
}

void imuScaleAccelMeasurements(int16_t rawAccel[3], float scaledAccel[3]) 
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleAccelMeasurement(&rawAccel[axis], &scaledAccel[axis]);
    }
}

void imuScaleAccelMeasurement(int16_t *rawAccel, float *scaledAccel)
{
    *scaledAccel = (float)(*rawAccel) / (float)(accelLSB);
}

void imuScaleMagMeasurements(int16_t rawMag[3], float scaledMag[3])
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleMagMeasurement(&rawMag[axis], &scaledMag[axis]);
    }
}

void imuScaleMagMeasurement(int16_t *rawMag, float *scaledMag)
{
    *scaledMag = (float)(*rawMag) * AK09916_UT_PER_LSB;
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