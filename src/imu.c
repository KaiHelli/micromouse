#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>     // if using printf
#include "i2c.h"
#include "imu.h"
#include "serialComms.h"
#include "IOconfig.h"

static uint8_t whoAmI = 0xFF;  // Buffer to store the read result
static uint8_t regAddr;

// -----------------------------------------------------------------------------
// This callback function is invoked from the I2C ISR after the transaction
// has completed (success or error).
// -----------------------------------------------------------------------------
static void imu_read_whoami_cb(bool success)
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
void imu_read_whoami(void)
{
    regAddr = ICM20948_PWR_MGMT_1;
    
    bool status = 0;
    
    // Switch to User Bank 1
    status |= imu_set_usr_bank(1);
    
    status |= putsI2C1(I2C_IMU_GYRO_ADDR, &regAddr, 1, &whoAmI, 1, imu_read_whoami_cb);
}

void imu_self_test(void) {
    
}

void imu_setup(void) {
    bool status = 0; // track the status of all operations
    
    // USR0 / PWR_MGMT_1
    // bit 6 -> 0 | wake from sleep mode
    // bit 3 -> 1 | disable temperature sensor
    // 0x41 on reset -> change to 0x05
    
    uint8_t value[] = {ICM20948_PWR_MGMT_1, 0x05};
    status |= putsI2C1Sync(I2C_IMU_GYRO_ADDR, value, 2, NULL, 0);

    // Switch to User Bank 2
    status |= imu_set_usr_bank(2);
    
    // USR2 / GYRO_SMPLRT_DIV
    // bit 7:0 -> Gyro sample rate divider
    // 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
    // 0x00 on reset
    
    // NOTE: left at 1.1kHz for now
    
    // USR2 / GYRO_CONFIG_1
    // bit 5:3 -> Gyro low pass filter configuration (see table 16)
    // bit 2:1 -> Gyro dynamic range selection (250, 500, 1000, 2000 dps)
    // bit 0 -> enable / disable low pass filter
    
    value[0] = ICM20948_GYRO_CONFIG_1;
    value[1] = 0b00000011;
    
    status |= putsI2C1Sync(I2C_IMU_GYRO_ADDR, value, 2, NULL, 0);
    
    // USR2 / GYRO_CONFIG_2
    // bit 5:3 -> X / Y / Z self-test enable
    // bit 2:0 -> averaging filter settings for low-power mode
    
    // USR2 / ACCEL_SMPLRT_DIV_1
    // bit 3:0 -> MSB for accelerometer sample rate division
    
    // USR2 / ACCEL_SMPLRT_DIV_2
    // bit 7:0 -> LSB for accelerometer sample rate division
    // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
    
    // Note: left at 1.125kHz for now
    
    // USR2 / ACCEL_CONFIG
    // bit 5:3 -> Accel low pass filter configuration (see table 18)
    // bit 2:1 -> Accel dynamic range selection (2g, 4g, 8g, 16g)
    // bit 0 -> enable / disable low pass filter
    
    // Note: left default (2G) for now
    
    // USR2 / ACCEL_CONFIG_2
    // bit 4:2 -> X / Y / Z self-test enable
    // bit 1:0 -> averaging filter settings for low-power mode
    
    if (status != 0) {
        putsUART1("Error setting up the IMU!\r\n");
    }
}

bool imu_set_usr_bank(uint8_t bank) {
    // USRX / REG_BANK_SEL (0x7F)
    // bits 5:4 -> set usr_bank 0-3
    bool status = 0;
    
    uint8_t value[] = {ICM20948_REG_BANK_SEL, bank << 4};
    status |= putsI2C1Sync(I2C_IMU_GYRO_ADDR, value, 1, NULL, 0);
    
    return status;
}