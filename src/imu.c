#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "imu.h"
#include "serialComms.h"
#include "IOconfig.h"

static uint8_t whoAmI = 0xFF;  // Buffer to store the read result

// Arrays holding LSB-per-unit for each selectable range:
static const float gyroLSBTable[]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const uint16_t accelLSBTable[] = { 16384, 8192, 4096, 2048 };

// These will store the selected LSB values:
static float gyroLSB  = 131.0f;  // default -- gyroLSBTable[0]
static float accelLSB = 16384;   // default -- accelLSBTable[0]

void imuReadGyroCb(bool success) {

    char measurementStr[50];
    
    float gyroMeasurements[3];
    
    imuScaleGyroMeasurements(rawGyroMeasurements, gyroMeasurements);
        
    snprintf(measurementStr, 50, "Gyroscope: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", gyroMeasurements[0], gyroMeasurements[1], gyroMeasurements[2]);
    putsUART1(measurementStr);
}

void imuReadAccelCb(bool success) {
    char measurementStr[50];

    float accelMeasurements[3];
        
    imuScaleAccelMeasurements(rawAccelMeasurements, accelMeasurements);

    snprintf(measurementStr, 50, "Accelerometer: X = %1.2f\tY = %1.2f\tZ = %1.2f\r\n", accelMeasurements[0], accelMeasurements[1], accelMeasurements[2]);
    putsUART1(measurementStr);
}

void imuReadGyro(void) {
    static uint8_t measurementRegisterStart = ICM20948_GYRO_XOUT_H;
    
    bool status = 0;

    // Switch to User Bank 0
    status |= imuSetUsrBank(0);
    
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) rawGyroMeasurements, 6, imuReadGyroCb);
}

void imuReadAccel(void) {
    static uint8_t measurementRegisterStart = ICM20948_ACCEL_XOUT_H;
    
    bool status = 0;

    // Switch to User Bank 0
    status |= imuSetUsrBank(0);
    
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, &measurementRegisterStart, 1, (uint8_t*) rawAccelMeasurements, 6, imuReadAccelCb);
}



// -----------------------------------------------------------------------------
// This callback function is invoked from the I2C ISR after the transaction
// has completed (success or error).
// -----------------------------------------------------------------------------
static void imuReadWhoAmICb(bool success)
{
    if (success)
    {
        // The whoAmI variable now holds the data read from the WHO_AM_I register.
        // According to most ICM-20948 datasheets, this should be 0xEA.
        char whoAmIStr[50];
        
        snprintf(whoAmIStr, 50, "ICM-20948 WHO_AM_I = 0x%02X\r\n", whoAmI);

        putsUART1(whoAmIStr);
    }
    else
    {
        // Handle I2C error (e.g., no sensor found, bus conflict, etc.)
        putsUART1("Asynchronous I2C read error!\r\n");
    }
}

// -----------------------------------------------------------------------------
// Starts an asynchronous read of the WHO_AM_I register. We send a single-byte
// register address, then read back one byte into 'whoAmI'.
// -----------------------------------------------------------------------------
void imuReadWhoAmI(void)
{
    static uint8_t regAddr = ICM20948_WHO_AM_I;
    
    bool status = 0;

    // Switch to User Bank 0
    status |= imuSetUsrBank(2);
    
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, &regAddr, 1, &whoAmI, 1, imuReadWhoAmICb);
}

void imuSelfTest(void)
{
    // Implementation placeholder
}

void imuSetup(GyroRange_t gyroRange, AccelRange_t accelRange)
{
    bool status = 0; // track the status of all operations
    
    // USR0 / PWR_MGMT_1
    // bit 6 -> 0 | wake from sleep mode
    // bit 3 -> 1 | disable temperature sensor
    // 0x41 on reset -> change to 0x05
    
    static uint8_t value1[] = { ICM20948_PWR_MGMT_1, 0x05 };
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, value1, 2, NULL, 0, NULL);

    // Switch to User Bank 2
    status |= imuSetUsrBank(2);
    
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
    uint8_t gyroConfig = ((uint8_t)gyroRange << 1);
    // Enable LPF in bit 0:
    gyroConfig |= 0x01;
    
    static uint8_t gyroData[] = { ICM20948_GYRO_CONFIG_1, 0x0 };
    gyroData[1] = gyroConfig;
    
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, gyroData, 2, NULL, 0, NULL);
    
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
    // For example, enable LPF in bit 0:
    accelConfig |= 0x01;
    
    static uint8_t accelData[] = { ICM20948_ACCEL_CONFIG, 0x0 };
    accelData[1] = accelConfig;
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, accelData, 2, NULL, 0, NULL);

    // Store the selected LSB
    accelLSB = accelLSBTable[accelRange];
    
    // USR2 / ACCEL_CONFIG_2
    // bit 4:2 -> X/Y/Z self-test enable
    // bit 1:0 -> averaging filter settings for low-power mode
    
    // MAG / CNTL2
    // bit 4:0 -> Magnetometer operation mode setting
    // default 0x0 (power down) -> set to 0b00010 (continuous measurement mode 1)
    
    if (status == 0) {
        putsUART1("IMU configured.\r\n");
    } else {
        putsUART1("Error setting up the IMU!\r\n");
    }
}

bool imuSetUsrBank(uint8_t bank)
{
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
    bool status = putsI2C1(I2C_IMU_GYRO_ADDR, bankRegs[bank], 2, NULL, 0, NULL);
    return status;
}

void imuScaleGyroMeasurements(const int16_t rawGyro[3], float scaledGyro[3])
{
    scaledGyro[0]  = (float)rawGyro[0]  / gyroLSB;
    scaledGyro[1]  = (float)rawGyro[1]  / gyroLSB;
    scaledGyro[2]  = (float)rawGyro[2]  / gyroLSB;
}

void imuScaleAccelMeasurements(const int16_t rawAccel[3], float scaledAccel[3]) 
{
    scaledAccel[0] = (float)rawAccel[0] / accelLSB;
    scaledAccel[1] = (float)rawAccel[1] / accelLSB;
    scaledAccel[2] = (float)rawAccel[2] / accelLSB;
}

float dpsToRadps(float dps)
{
    return dps * (M_PI / 180.0f);
}