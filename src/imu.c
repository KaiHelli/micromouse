#include "imu.h"
#include "i2c.h"
#include "uart.h"
#include "odometry.h"
#include "IOconfig.h"

#include "clock.h" // Has to be imported before libpic30, as it defines FCY
#include <libpic30.h>

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define IMU_CHECK_BANK_SYNC(bank)         \
    if (!imuSetUsrBankSync(bank)) {       \
        return false;                     \
    }

// Arrays holding LSB-per-unit for each selectable range:
static const float gyroLSBTable[]  = { 131.0f, 65.5f, 32.8f, 16.4f };
static const uint16_t accelLSBTable[] = { 16384, 8192, 4096, 2048 };

// These will store the selected LSB values:
static float gyroLSB  = 131.0f;  // default -- gyroLSBTable[0]
static uint16_t accelLSB = 16384;   // default -- accelLSBTable[0]

// Save the current FIFO config
static FifoConfig_t fifoConfig;
static uint16_t fifoBytesPerDataset = 0;
static bool imuMagDirectCom = 0;

/*
 *  General async callback to alert something went wrong
 */
void imuCommCb(bool success) {
    if (!success)
    {
        putsUART1Str("Asynchronous IMU error!\r\n");
    }
}

/*
 * Switch user bank
 */

// Track the user bank we are currently on
static volatile uint8_t currentBank = 0;

void imuSetUsrBank(uint8_t bank)
{
    if (bank >= 4) {
        return;
    }
    
    // Check if we actually have to do something.
    if (currentBank == bank) {
        return;
    }
    
    // Prepare a lookup table for bank 0..3
    // bits 5:4 hold the user bank ID.
    static uint8_t bankRegs[4][2] =
    {
        { ICM20948_REG_BANK_SEL, 0 << 4 }, // Bank 0
        { ICM20948_REG_BANK_SEL, 1 << 4 }, // Bank 1
        { ICM20948_REG_BANK_SEL, 2 << 4 }, // Bank 2
        { ICM20948_REG_BANK_SEL, 3 << 4 }  // Bank 3
    };
    
    // Perform the async I2C write using the correct buffer
    // (the data remains valid because `bankRegs` is static)
    putsI2C1(I2C_IMU_GYRO_ADDR, bankRegs[bank], 2, NULL, 0, imuCommCb);
    
    // We assume the write completes... not perfect though but avoids race
    // conditions with the if check above.
    currentBank = bank;
}

bool imuSetUsrBankSync(uint8_t bank)
{
    if (bank >= 4) {
        return false;
    }
    
    // Check if we actually have to do something.
    if (currentBank == bank) {
        return true;
    }
    
    uint8_t bankReg[] = {ICM20948_REG_BANK_SEL, bank << 4};
    
    // Perform the async I2C write using the correct buffer
    // (the data remains valid because `bankRegs` is static)
    bool status = putsI2C1Sync(I2C_IMU_GYRO_ADDR, bankReg, 2, NULL, 0);
    
    if (status) {
        currentBank = bank;
    }
    
    return status;
}

/*
 * Read / write helpers
 */
void imuWrite(uint8_t *wBuf, uint16_t wLen, I2CCallback_t cb) {
    if (!cb) {
        cb = imuCommCb;
    }
    
    putsI2C1(I2C_IMU_GYRO_ADDR, wBuf, wLen, NULL, 0, cb);
}

bool imuWrite8Sync(uint8_t reg, uint8_t val) {
    uint8_t buf[] = { reg, val };
    
    return putsI2C1Sync(I2C_IMU_GYRO_ADDR, buf, 2, NULL, 0);
}

bool imuWrite16Sync(uint8_t reg, uint16_t val, bool littleEndian) {
    uint8_t buf[3];
    buf[0] = reg;

    if (littleEndian) {
        buf[1] = LOW_BYTE(val);
        buf[2] = HIGH_BYTE(val);
    } else {
        buf[1] = HIGH_BYTE(val);
        buf[2] = LOW_BYTE(val);
    }

    return putsI2C1Sync(I2C_IMU_GYRO_ADDR, buf, 3, NULL, 0);
}

bool imuWriteBufSync(uint8_t *wBuf, uint16_t len) {
    return putsI2C1Sync(I2C_IMU_GYRO_ADDR, wBuf, len, NULL, 0);
}

void imuRead(uint8_t *wBuf, uint16_t wLen, uint8_t *rBuf, uint16_t rLen, I2CCallback_t cb) {
    if (!cb) {
        cb = imuCommCb;
    }
    
    putsI2C1(I2C_IMU_GYRO_ADDR, wBuf, wLen, rBuf, rLen, cb);
}

bool imuReadSync(uint8_t reg, uint8_t *val, uint16_t len) {
    uint8_t i2cData[] = { reg };
    
    return putsI2C1Sync(I2C_IMU_GYRO_ADDR, i2cData, 1, val, len);
}

bool imuSetBitSync(uint8_t reg, uint8_t bit, bool state) {
    if (bit > 7) {
        return false;  // invalid bit index
    }

    uint8_t val;
    bool status = imuReadSync(reg, &val, 1);
    
    if (!status) {
        return false;  // read failed
    }

    if (state) {
        val |= (1 << bit);      // set the bit
    } else {
        val &= ~(1 << bit);     // clear the bit
    }

    // write modified value back to the same register
    status = imuWrite8Sync(reg, val);
    return status;
}

bool imuMagWriteSync(uint8_t reg, uint8_t val, bool directCom) {
    bool status = true;
    
    // Communicate with the IMU directly instead of indirectly over the main chip
    if (directCom) {
        uint8_t i2cData[] = {reg, val};

        status &= putsI2C1Sync(I2C_IMU_MAG_ADDR, i2cData, 2, NULL, 0);
    } else {
        uint8_t bankBefore = currentBank;
        IMU_CHECK_BANK_SYNC(3);

        // Set slave address to write to
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_ADDR, I2C_IMU_MAG_ADDR);
        // Set value to write
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_DO, val);
        // Set register to write to
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_REG, reg);
        // Start transmission
        status &= imuSetBitSync(ICM20948_I2C_SLV4_CTRL, 7, true);
        // Wait for completion
        IMU_CHECK_BANK_SYNC(0);
        uint8_t txState = 0;
        do {
            status &= imuReadSync(ICM20948_I2C_MST_STATUS, &txState, 1);
        }
        while (status && !(txState & (1 << 6)));

        // Is NACK set -> transaction failed
        status &= !(txState & (1 << 4));
                
        // Restore bank
        IMU_CHECK_BANK_SYNC(bankBefore);
    }
    
    return status;
}

bool imuMagReadSync(uint8_t reg, uint8_t *val, uint16_t len, bool directCom) {
    bool status = true;
    
    if (directCom) {
        uint8_t i2cData[] = { reg };

        status &= putsI2C1Sync(I2C_IMU_MAG_ADDR, i2cData, 1, val, len);
    } else {
        // Only a single byte is supported to read indirectly
        if (len > 1) {
            return false;
        }
        
        uint8_t bankBefore = currentBank;
        IMU_CHECK_BANK_SYNC(3);

        // Set slave address to read from
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_ADDR, (1 << 7) | I2C_IMU_MAG_ADDR);
        // Set register to read from
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_REG, reg);
        // Start transmission
        status &= imuSetBitSync(ICM20948_I2C_SLV4_CTRL, 7, true);
        // Wait for completion
        uint8_t txState = 0;
        IMU_CHECK_BANK_SYNC(0);
        do {
            status &= imuReadSync(ICM20948_I2C_MST_STATUS, &txState, 1);
        }
        while (status && !(txState & (1 << 6)));

        // Is NACK set -> transaction failed
        status &= !(txState & (1 << 4));
        
        IMU_CHECK_BANK_SYNC(3);
        status &= imuReadSync(ICM20948_I2C_SLV4_DI, val, len);
        
        // Restore bank
        IMU_CHECK_BANK_SYNC(bankBefore);
    }
    
    return status;
}

bool imuMagSetBitSync(uint8_t reg, uint8_t bit, bool state, bool directCom) {
    if (bit > 7) {
        return false;  // invalid bit index
    }

    uint8_t val;
    bool status = imuMagReadSync(reg, &val, 1, directCom);
    
    if (!status) {
        return false;  // read failed
    }

    if (state) {
        val |= (1 << bit);      // set the bit
    } else {
        val &= ~(1 << bit);     // clear the bit
    }

    // write modified value back to the same register
    status = imuMagWriteSync(reg, val, directCom);
    return status;
}

/*
 * Reset
 */
bool imuResetSync(void) {
    bool status = true;
    
    IMU_CHECK_BANK_SYNC(0);
    
    // USR0 / PWR_MGMT_1
    // bit 7 -> 1 | reset IMU
    status &= imuSetBitSync(ICM20948_PWR_MGMT_1, 7, true);
    
    __delay_ms(5);
    
    return status;
}

bool imuI2CMasterResetSync(void) {
    bool status = true;
    
    IMU_CHECK_BANK_SYNC(0);
    
    // USR0 / USER_CTRL
    // bit 1 -> 1 | reset I2C master
    status &= imuSetBitSync(ICM20948_USER_CTRL, 1, true);
    
    __delay_ms(5);
    
    return status;
}

bool imuMagResetSync(bool directCom) {
    bool status = true;
    
    // MAG / CNTL3
    // bit 0 -> reset magnetometer
    // 0x00 on reset -> change to 0x01
    status &= imuMagSetBitSync(AK09916_CNTL3, 0, true, directCom);
    
    __delay_ms(5);
    
    return status;
}

void imuFifoReset(void) {
    imuSetUsrBank(0);
    
    static uint8_t bufAssert[] = { ICM20948_FIFO_RST, 0x1F};
    static uint8_t bufDeassert[] = { ICM20948_FIFO_RST, 0x1E};
    imuWrite(bufAssert, 2, imuCommCb);
    imuWrite(bufDeassert, 2, imuCommCb);
}

bool imuFifoResetSync(void) {
    bool status = true;
    
    IMU_CHECK_BANK_SYNC(0);
    
    // USR0 / FIFO_RST
    // bit 0 -> reset FIFO on assert / de-assert
    status &= imuWrite8Sync(ICM20948_FIFO_RST, 0x1F);
    status &= imuWrite8Sync(ICM20948_FIFO_RST, 0x1E);
    
    return status;
}

/*
 * Setup
 */
bool imuSetupMagDataTransfer(uint8_t reg, uint8_t len, bool swap, bool oddAddrFirst) {
    bool status = true;
    
    uint8_t bankBefore = currentBank;
    IMU_CHECK_BANK_SYNC(3);
    
    // Set ODR to 1125/2^3 = 140,625 Hz matching roughly the 100Hz data rate of the magnetometer
    // In theory, only has effect if gyroscope and accelerometer are disabled. But in practice
    // it works.
    status &= imuWrite8Sync(ICM20948_I2C_MST_ODR_CONFIG, 0x03);
    
    // Set slave address to read from
    status &= imuWrite8Sync(ICM20948_I2C_SLV0_ADDR, (1 << 7) | I2C_IMU_MAG_ADDR);
    // Set register to read from
    status &= imuWrite8Sync(ICM20948_I2C_SLV0_REG, reg);
    
    uint8_t readConfig = 0x00;
    // Read data at sample rate to EXT_SENS_DATA_00
    readConfig |= 1 << 7;
    // Swap bytes if enabled
    readConfig |= swap << 6;
    // Set whether bytes with odd addresses for the first byte are grouped
    readConfig |= oddAddrFirst << 4;
    // Set number of bytes to be read
    readConfig |= len & 0xF;
    
    status &= imuWrite8Sync(ICM20948_I2C_SLV0_CTRL, readConfig);
    
    IMU_CHECK_BANK_SYNC(bankBefore);
    
    return status;
}

bool imuSetup(GyroRange_t gyroRange, AccelRange_t accelRange, MagMode_t magMode, TempMode_t tempMode, FifoConfig_t fifoCfg) {
    __delay_ms(1000);
    
    bool status = true; // track the status of all operations
    
    // Patch FIFO config
    fifoCfg.mag &= magMode != MAG_MODE_OFF;
    fifoCfg.temp &= tempMode != TEMP_OFF;
    
    // In case we don't use the FIFO for the Magnetometer, we are going to directly
    // communicate with the magnetometer instead of going through the I2C of the 
    // main chip
    imuMagDirectCom = !fifoCfg.mag;
    
    /*
     * MAIN IMU SETUP
     */
    
    // Reset IMU
    status &= imuResetSync();
    
    IMU_CHECK_BANK_SYNC(0);
    
    // USR0 / PWR_MGMT_1
    // bit 6 -> 0 | wake from sleep mode
    // bit 3 -> 1 | enable/disable temperature sensor
    // 0x41 on reset -> change to 0x01 / 0x09
    status &= imuWrite8Sync(ICM20948_PWR_MGMT_1, 0x01 | (tempMode << 3));
    
    IMU_CHECK_BANK_SYNC(2);
    
    // USR2 / ODR_ALIGN_EN
    // bit 0 -> 1 | align sampling rates of sensors
    // 0x00 on reset -> change to 0x01
    status &= imuWrite8Sync(ICM20948_ODR_ALIGN_EN, 0x01);
    
    /*
     * Gyro Data Rate, Filter and Averaging
     * GYRO_FCHOICE -> 1 (enable gyro DLPF)
     * GYRO_DLPFCFG -> 4 (mid LPF config)
     * GYRO_SMPLRT_DIV -> 2 (1125Hz/(1+GYRO_SMPLRT_DIV) -> ODR of 562.5 Hz
     * GYRO_AVGCFG -> 0 (1x averaging in low-power mode)
     */
    
    // USR2 / GYRO_SMPLRT_DIV
    // bit 7:0 -> Gyro sample rate divider
    // 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
    // 0x00 on reset
    status &= imuWrite8Sync(ICM20948_GYRO_SMPLRT_DIV, 0x01);
    
    // USR2 / GYRO_CONFIG_1
    // bit 5:3 -> Gyro low pass filter configuration (see table 16)
    // bit 2:1 -> Gyro dynamic range selection (250, 500, 1000, 2000 dps)
    // bit 0 -> enable / disable low pass filter
    
    // Set gyro range selection.
    uint8_t gyroConfig = (uint8_t)gyroRange << 1;
    // Set low pass filter configuration
    gyroConfig |= 4 << 3;
    // Enable LPF in bit 0:
    gyroConfig |= 0x01;
    
    status &= imuWrite8Sync(ICM20948_GYRO_CONFIG_1, gyroConfig);
    
    // Store the selected LSB
    gyroLSB = gyroLSBTable[gyroRange];
    
    // USR2 / GYRO_CONFIG_2
    // bit 5:3 -> X/Y/Z self-test enable
    // bit 2:0 -> averaging filter settings for low-power mode
    status &= imuWrite8Sync(ICM20948_GYRO_CONFIG_2, 0x00);
    
    /*
     * Accel Data Rate, Filter and Averaging
     * ACCEL_FCHOICE -> 1 (enable accel DLPF)
     * ACCEL_DLPFCFG -> 2 (low LPF config)
     * ACCEL_SMPLRT_DIV -> 2 (1.125kHz/(1+ACCEL_SMPLRT_DIV) -> ODR of 562.5 Hz
     * ACCEL_AVGCFG -> 0 (1x averaging in low-power mode)
     */
    
    // USR2 / ACCEL_SMPLRT_DIV_1
    // bit 3:0 -> MSB for accelerometer sample rate division
    status &= imuWrite8Sync(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
    
    // USR2 / ACCEL_SMPLRT_DIV_2
    // bit 7:0 -> LSB for accelerometer sample rate division
    // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
    status &= imuWrite8Sync(ICM20948_ACCEL_SMPLRT_DIV_2, 0x01);
    
    // USR2 / ACCEL_CONFIG
    // bit 5:3 -> Accel low pass filter configuration (see table 18)
    // bit 2:1 -> Accel dynamic range selection (2g, 4g, 8g, 16g)
    // bit 0 -> enable / disable low pass filter
    
    // Set accelerometer range selection.
    uint8_t accelConfig = (uint8_t)accelRange << 1;
    // Set low pass filter configuration
    accelConfig |= 2 << 3;
    // Enable LPF in bit 0:
    accelConfig |= 0x01;
    
    status &= imuWrite8Sync(ICM20948_ACCEL_CONFIG, accelConfig);

    // Store the selected LSB
    accelLSB = accelLSBTable[accelRange];
    
    // USR2 / ACCEL_CONFIG_2
    // bit 4:2 -> X/Y/Z self-test enable
    // bit 1:0 -> averaging filter settings for low-power mode
    status &= imuWrite8Sync(ICM20948_ACCEL_CONFIG_2, 0x00);
    
    if (imuMagDirectCom) {
        IMU_CHECK_BANK_SYNC(0);

        // USR0 / INT_PIN_CFG
        // bit 1 -> 1 | bridge auxiliary I2C bus with main bus
        // 0x00 on reset -> change to 0x02
        status &= imuWrite8Sync(ICM20948_INT_PIN_CFG, 0x02);
    } else {
        IMU_CHECK_BANK_SYNC(0);
        
        // USR0 / LP_CONFIG
        // bit 6 -> 0 | disable I2C master duty cycled mode
        // status &= imuWrite8Sync(ICM20948_LP_CONFIG, 0x00);
        
        // USR0 / USER_CTRL
        // bit 5 -> 1 | enable I2C master for magnetometer access
        // 0x00 on reset -> change to 0x20
        status &= imuWrite8Sync(ICM20948_USER_CTRL, (1 << 5));
        
        IMU_CHECK_BANK_SYNC(3);

        // USR3 / I2C_MST_CTRL
        // bit 4 -> set to 1 for having a stop between reads
        // bit 3:0 -> set I2C master clock frequency to targeted 400kHz (see table 23)
        uint8_t i2cCtrl = (1 << 4) | 0x07;
        status &= imuWrite8Sync(ICM20948_I2C_MST_CTRL, i2cCtrl);
        
        // TODO: Describe why needed
        status &= imuWrite8Sync(ICM20948_I2C_MST_DELAY_CTRL, 0x01);
        status &= imuWrite8Sync(ICM20948_I2C_SLV4_CTRL, 0x00);
        
        // Reset I2C Master
        // status &= imuI2CMasterResetSync();
    }
    
    __delay_ms(10);
    
    /*
     * MAIN MAGNETOMETER SETUP
     */
    
    // Reset magnetometer
    status &= imuMagResetSync(imuMagDirectCom);
    
    // MAG / CNTL2
    // bit 4:0 -> Magnetometer operation mode setting
    status &= imuMagWriteSync(AK09916_CNTL2, magMode, imuMagDirectCom);
    
    // Setup copying if magnetometer is enabled
    // Swap bytes to be consistent with endianness on IMU itself
    // We read 9 bytes:
    // Read status 1 to check data ready
    // Read 6 Bytes measurement data
    // Read 1 Byte to jump to status 2 register
    // Read status 2 to check overflow and unblock measurement registers
    if (magMode != MAG_MODE_OFF && !imuMagDirectCom) {
        imuSetupMagDataTransfer(AK09916_ST1, 0x09, false, false);
    }
    
    /*
     * IMU FIFO SETUP
     */
    
    // Store globally
    fifoConfig = fifoCfg;
    fifoBytesPerDataset = 0;

    if (fifoConfig.accel || fifoConfig.gyro || fifoConfig.mag || fifoConfig.temp) {
        IMU_CHECK_BANK_SYNC(0);
    
        // USR0 / USER_CTRL
        // bit 6 -> 1 | enable FIFO
        status &= imuSetBitSync(ICM20948_USER_CTRL, 6, true);
        
        // USR0 / FIFO_MODE
        // bit 4:0 -> mode is set to "stream" by default -> nothing to set
        
        if (fifoConfig.accel) {
            // USR0 / FIFO_EN_2
            // bit 4 -> Write ACCEL data to the FIFO at sample rate
            status &= imuSetBitSync(ICM20948_FIFO_EN_2, 4, true);
            fifoBytesPerDataset += 6;
        }
        
        if (fifoConfig.gyro) {
            // USR0 / FIFO_EN_2
            // bit 3:1 -> Write GYRO data to the FIFO at sample rate
            status &= imuSetBitSync(ICM20948_FIFO_EN_2, 3, true);
            status &= imuSetBitSync(ICM20948_FIFO_EN_2, 2, true);
            status &= imuSetBitSync(ICM20948_FIFO_EN_2, 1, true);
            fifoBytesPerDataset += 6;
        }
        
        if (fifoConfig.mag) {
            // USR0 / FIFO_EN_1
            // bit 0 -> Write SLV_0 data (mag) to the FIFO at sample rate
            status &= imuSetBitSync(ICM20948_FIFO_EN_1, 0, true);
            fifoBytesPerDataset += 9;
        }
        
        if (fifoConfig.temp) {
            // USR0 / FIFO_EN_2
            // bit 0 -> Write TEMP data to the FIFO at sample rate
            status &= imuSetBitSync(ICM20948_FIFO_EN_2, 0, true);
            fifoBytesPerDataset += 2;
        }
    }
    
    status &= imuFifoResetSync();
    
    // Wait for settings to take effect.
    __delay_ms(100);
    
    /*
     * SANITY CHECK
     */
    
    IMU_CHECK_BANK_SYNC(0);
    
    // Check "WHO AM I" values for correctness.
    uint8_t imuWhoAmI;
    status &= imuReadSync(ICM20948_WHO_AM_I, &imuWhoAmI, 1);
    
    
    uint8_t imuMagWhoAmI;
    status &= imuMagReadSync(AK09916_WHO_AM_I, &imuMagWhoAmI, 1, imuMagDirectCom);
    
    if (status && imuWhoAmI == ICM20948_ID && imuMagWhoAmI == AK09916_ID) {
        putsUART1Str("IMU configured.\r\n");
    } else {
        putsUART1Str("Error setting up the IMU!\r\n");
    }
    
    return status;
}

bool imuCalibrateGyro() {
    putsUART1Str("Calibrating IMU Gyroscope. Hold still.\r\n");
    
    const uint16_t numSamples = 500;
    uint16_t numMeasurements = 0;
    
    // Use float for accuracy of running average calculations
    float runningAverage[3] = {0.0f, 0.0f, 0.0f};

    while (numMeasurements < numSamples) {
        int16_t rawGyroMeasurements[3];

        bool readStatus = imuReadGyroSync(rawGyroMeasurements, NULL, false);

        if (!readStatus) {
            continue;
        }

        // Incremental running average calculation
        for (uint8_t axis = 0; axis < 3; axis++) {
            runningAverage[axis] = ((runningAverage[axis] * numMeasurements) + rawGyroMeasurements[axis])
                                    / (numMeasurements + 1);
        }

        numMeasurements++;

        // At a sampling rate of 1.1kHz, wait 10ms before next reading
        __delay_ms(10);
    }
    
    // After calibration, offsets are available in runningAverage[]
    // (Write these values to the IMU offset registers as needed)
    IMU_CHECK_BANK_SYNC(2);
    
    // Round float values to int16_t before writing offsets
    int16_t gyroOffsets[3];
    
    // The offsets have to be set in the +-1000 dps sensitivity range. Therefore,
    // we might have to scale it.
    float sensitivityScaling = gyroLSB / gyroLSBTable[GYRO_RANGE_1000DPS];
    
    char calibrationStr[40];
    snprintf(calibrationStr, sizeof(calibrationStr), "Averages: %.2f, %.2f, %.2f\r\n", runningAverage[0], runningAverage[1], runningAverage[2]);
    putsUART1Str(calibrationStr);
    
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
    
    status &= imuWriteBufSync(offsetValues, 7);
    
    if (status) {
        putsUART1Str("Gyroscope calibrated.\r\n");
    } else {
        putsUART1Str("Error calibrating the gyroscope!\r\n");
    }
    
    return status;
}

bool imuCalibrateAccel() {
    putsUART1Str("Calibrating IMU Accelerometer. Hold still.\r\n");

    // Read factory calibration
    // Seems to be: X = 1190 Y = -1365 Z = -269
    int16_t accelOffsets[3] = {0, 0, 0};
    
    const uint8_t xOffsetRegisterStart = ICM20948_XA_OFFSET_H;
    const uint8_t yOffsetRegisterStart = ICM20948_YA_OFFSET_H;
    const uint8_t zOffsetRegisterStart = ICM20948_ZA_OFFSET_H;
    
    bool status = 1;
    
    // Switch to User Bank 1
    IMU_CHECK_BANK_SYNC(1);

    // Unfortunately addresses of registers are not consecutive.
    status &= imuReadSync(xOffsetRegisterStart, (uint8_t*) &accelOffsets[0], 2);
    status &= imuReadSync(yOffsetRegisterStart, (uint8_t*) &accelOffsets[1], 2);
    status &= imuReadSync(zOffsetRegisterStart, (uint8_t*) &accelOffsets[2], 2);
    
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

        bool readStatus = imuReadAccelSync(rawAccelMeasurements, NULL, true);

        if (!readStatus) {
            continue;
        }
        
        float calibratedAccelMeasurements[3];
        imuCalibrateAccelMeasurements(rawAccelMeasurements, calibratedAccelMeasurements);
        
        // Revert mapping to IMU frame
        calibratedAccelMeasurements[0] = -calibratedAccelMeasurements[0];
        calibratedAccelMeasurements[1] = -calibratedAccelMeasurements[1];

        // Incremental running average calculation
        for (uint8_t axis = 0; axis < 3; axis++) {
            runningAverage[axis] = ((runningAverage[axis] * numMeasurements) + calibratedAccelMeasurements[axis])
                                    / (numMeasurements + 1);
        }

        numMeasurements++;

        // At a sampling rate of 1.125kHz, wait 10ms before next reading
        __delay_ms(10);
    }
    
    // We expect gravity to have 1G.
    runningAverage[2] -= (float) accelLSB;
    
    // After calibration, offsets are available in runningAverage[]
    // (Write these values to the IMU offset registers as needed)
    IMU_CHECK_BANK_SYNC(1);
    
    // The offsets have to be set in the +-16 G sensitivity range. Therefore,
    // we might have to scale it.
    float sensitivityScaling = accelLSB / accelLSBTable[ACCEL_RANGE_16G];
    
    // TODO: Magic factor of 2, that we are currently unsure where it stems from.
    sensitivityScaling *= 2;
    
    char calibrationStr[40];
    snprintf(calibrationStr, sizeof(calibrationStr), "Averages: %.2f, %.2f, %.2f\r\n", runningAverage[0], runningAverage[1], runningAverage[2]);
    putsUART1Str(calibrationStr);
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        accelOffsets[axis] += (int16_t)roundf(-runningAverage[axis] / sensitivityScaling);
        
        // Add temperature bit back in
        accelOffsets[axis] <<= 1;
        accelOffsets[axis] |= temperatureBit[axis];
    }
    
    status &= imuWrite16Sync(xOffsetRegisterStart, accelOffsets[0], false);
    status &= imuWrite16Sync(yOffsetRegisterStart, accelOffsets[1], false);
    status &= imuWrite16Sync(zOffsetRegisterStart, accelOffsets[2], false);
    
    if (status) {
        putsUART1Str("Accelerometer calibrated.\r\n");
    } else {
        putsUART1Str("Error calibrating the IMU!\r\n");
    }
    
    return status;
}

/*
 * Remapping helpers
 */

/*
 * remapGyroAxes(src, dst):
 *   Remap raw gyro counts from sensor -> robot frame.
 *   Sensor [+X CW@Z, +Y CCW@Y, +Z CCW@Z]
 *   -> Robot  [+X CW@X, +Y CW@Y, +Z CW@Z]
 */
static inline void remapGyroAxesFloat(float src[3], float dst[3]) {
    float tmp[3];
    
    tmp[0] =  src[0];
    tmp[1] = -src[1];
    tmp[2] = -src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}

static inline void remapGyroAxes(int16_t src[3], int16_t dst[3]) {
    int16_t tmp[3];
    
    tmp[0] =  src[0];
    tmp[1] = -src[1];
    tmp[2] = -src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}

/*
 * remapAccelAxes(src, dst):
 *   Remap raw accel counts from sensor -> robot frame.
 *   Sensor [+X left, +Y back, +Z up]
 *   -> Robot  [+X right, +Y forward, +Z up]
 */
static inline void remapAccelAxesFloat(float src[3], float dst[3]) {
    float tmp[3];
    
    tmp[0] = -src[0];
    tmp[1] = -src[1];
    tmp[2] =  src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}

static inline void remapAccelAxes(int16_t src[3], int16_t dst[3]) {
    int16_t tmp[3];
    
    tmp[0] = -src[0];
    tmp[1] = -src[1];
    tmp[2] =  src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}

/*
 * remapMagAxes(src, dst):
 *   Remap raw mag counts from sensor -> robot frame.
 *   Sensor [+X left, +Y forward, +Z down]
 *   -> Robot  [+X right, +Y forward, +Z up]
 */
static inline void remapMagAxesFloat(float src[3], float dst[3]) {
    float tmp[3];
    
    tmp[0] = -src[0];
    tmp[1] =  src[1];
    tmp[2] = -src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}


static inline void remapMagAxes(int16_t src[3], int16_t dst[3]) {
    int16_t tmp[3];
    
    tmp[0] = -src[0];
    tmp[1] =  src[1];
    tmp[2] = -src[2];
    
    dst[0] = tmp[0];
    dst[1] = tmp[1];
    dst[2] = tmp[2];
}

/*
 * Scaling helpers
 */

void imuScaleGyroMeasurementsFloat(float rawGyro[3], float scaledGyro[3])
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleGyroMeasurementFloat(&rawGyro[axis], &scaledGyro[axis]);
    }
}

void imuScaleGyroMeasurementFloat(float *rawGyro, float *scaledGyro)
{
    *scaledGyro = (*rawGyro) / gyroLSB;
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

void imuScaleAccelMeasurementsFloat(float rawAccel[3], float scaledAccel[3]) 
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleAccelMeasurementFloat(&rawAccel[axis], &scaledAccel[axis]);
    }
}

void imuScaleAccelMeasurementFloat(float *rawAccel, float *scaledAccel)
{
    *scaledAccel = (*rawAccel) / (float)(accelLSB);
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

void imuScaleMagMeasurementsFloat(float rawMag[3], float scaledMag[3])
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        imuScaleMagMeasurementFloat(&rawMag[axis], &scaledMag[axis]);
    }
}

void imuScaleMagMeasurementFloat(float *rawMag, float *scaledMag)
{
    *scaledMag = (float)(*rawMag) * AK09916_UT_PER_LSB;
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

void imuScaleTempMeasurementsFloat(float *rawTemp, float *scaledTemp)
{
    *scaledTemp = (*rawTemp / ICM20948_LSB_PER_C) + 21;
}

void imuScaleTempMeasurements(int16_t *rawTemp, float *scaledTemp)
{
    *scaledTemp = ((float)*rawTemp / ICM20948_LSB_PER_C) + 21;
}

/*
 * Calibration helpers
 */

// Total Field in nT in Garching (at 48.26274629587023, 11.667870576826944, 482m Elevation)
// https://www.ncei.noaa.gov/products/world-magnetic-model
const float totalField = 47738.8;
const float totalFieldLSB = 318.2586;

// External magnetometer and accelerometer calibration constants
const float imuAccelBias[3] = { 26.24 , 233.71 , 273.55 };
const float imuAccelScale[3][3] = {{ 0.9992 , 1e-05 , 0.00196 },
                                 { 1e-05 , 0.99849 , -0.00139 },
                                 { 0.00196 , -0.00139 , 0.9951 }};

const float imuMagBias[3] = { 549.1, -240.26, 67.64 };
const float imuMagScale[3][3] = {{ 1.28313 , 0.03529 , 0.00263 },
                                 { 0.03529 , 1.21936 , -0.00232 },
                                 { 0.00263 , -0.00232 , 1.19005 }};

void imuCalibrateAccelMeasurementsFloat(float *rawAccel, float *scaledAccel)
{
    float tempAccel[3];
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        tempAccel[axis] = (rawAccel[axis] - imuAccelBias[axis]);
    }
    
    scaledAccel[0] = imuAccelScale[0][0] * tempAccel[0] + imuAccelScale[0][1] * tempAccel[1] + imuAccelScale[0][2] * tempAccel[2];
    scaledAccel[1] = imuAccelScale[1][0] * tempAccel[0] + imuAccelScale[1][1] * tempAccel[1] + imuAccelScale[1][2] * tempAccel[2];
    scaledAccel[2] = imuAccelScale[2][0] * tempAccel[0] + imuAccelScale[2][1] * tempAccel[1] + imuAccelScale[2][2] * tempAccel[2];    
}

void imuCalibrateAccelMeasurements(int16_t *rawAccel, float *scaledAccel)
{
    float tempAccel[3];
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        tempAccel[axis] = ((float) rawAccel[axis] - imuAccelBias[axis]);
    }
    
    scaledAccel[0] = imuAccelScale[0][0] * tempAccel[0] + imuAccelScale[0][1] * tempAccel[1] + imuAccelScale[0][2] * tempAccel[2];
    scaledAccel[1] = imuAccelScale[1][0] * tempAccel[0] + imuAccelScale[1][1] * tempAccel[1] + imuAccelScale[1][2] * tempAccel[2];
    scaledAccel[2] = imuAccelScale[2][0] * tempAccel[0] + imuAccelScale[2][1] * tempAccel[1] + imuAccelScale[2][2] * tempAccel[2];    
}

void imuCalibrateMagMeasurementsFloat(float *rawMag, float *scaledMag)
{
    float tempMag[3];
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        tempMag[axis] = (rawMag[axis] - imuMagBias[axis]);
    }
    
    scaledMag[0] = imuMagScale[0][0] * tempMag[0] + imuMagScale[0][1] * tempMag[1] + imuMagScale[0][2] * tempMag[2];
    scaledMag[1] = imuMagScale[1][0] * tempMag[0] + imuMagScale[1][1] * tempMag[1] + imuMagScale[1][2] * tempMag[2];
    scaledMag[2] = imuMagScale[2][0] * tempMag[0] + imuMagScale[2][1] * tempMag[1] + imuMagScale[2][2] * tempMag[2];    
}

void imuCalibrateMagMeasurements(int16_t *rawMag, float *scaledMag)
{
    float tempMag[3];
    
    for (uint8_t axis = 0; axis < 3; axis++) {
        tempMag[axis] = ((float) rawMag[axis] - imuMagBias[axis]);
    }
    
    scaledMag[0] = imuMagScale[0][0] * tempMag[0] + imuMagScale[0][1] * tempMag[1] + imuMagScale[0][2] * tempMag[2];
    scaledMag[1] = imuMagScale[1][0] * tempMag[0] + imuMagScale[1][1] * tempMag[1] + imuMagScale[1][2] * tempMag[2];
    scaledMag[2] = imuMagScale[2][0] * tempMag[0] + imuMagScale[2][1] * tempMag[1] + imuMagScale[2][2] * tempMag[2];    
}

/*
 * Obtain Measurements directly
 */

volatile int16_t rawGyroMeasurements[3];
volatile int16_t rawAccelMeasurements[3];
volatile int16_t rawMagMeasurements[3];
volatile int16_t rawTempMeasurement;

// Local variables that are "unstable".
volatile int16_t localGyroMeasurements[3];
volatile int16_t localAccelMeasurements[3];
volatile int16_t localMagMeasurements[3];
volatile int16_t localTempMeasurement;

// Register start addresses for each sensor type
static uint8_t imuSensorRegStart[] = {
    [IMU_SENSOR_GYRO]  = ICM20948_GYRO_XOUT_H,   // Gyro X high byte
    [IMU_SENSOR_ACCEL] = ICM20948_ACCEL_XOUT_H,  // Accel X high byte
    [IMU_SENSOR_MAG]   = 0,                      // Depends on direct or indirect comm -> handled at runtime by functions below
    [IMU_SENSOR_TEMP]  = ICM20948_TEMP_OUT_H     // Temp high byte
};

static const uint8_t magDirectReg   = AK09916_XOUT_L;
static const uint8_t magIndirectReg = ICM20948_EXT_SENS_DATA_01;

static inline const uint8_t* getImuRegPtr(ImuSensor_t sensor) {
    if (sensor == IMU_SENSOR_MAG) {
        return imuMagDirectCom ? &magDirectReg : &magIndirectReg;
    } else {
        return &imuSensorRegStart[sensor];
    }
}

// Number of data bytes for each sensor output
static uint8_t imuSensorLengths[] = {
    [IMU_SENSOR_GYRO]  = 6,  // 3 axes × 2 bytes
    [IMU_SENSOR_ACCEL] = 6,  // 3 axes × 2 bytes
    [IMU_SENSOR_MAG]   = 6,  // 3 axes × 2 bytes
    [IMU_SENSOR_TEMP]  = 2   // 1 reading × 2 bytes
};

/**
 * @brief Generic asynchronous read from IMU or magnetometer
 */
// Static constants for magnetometer register starts (direct vs. indirect)

void imuGenericRead(ImuSensor_t sensor, uint8_t *buffer, I2CCallback_t cb) {
    // Determine I2C address
    uint8_t addr = (sensor == IMU_SENSOR_MAG && imuMagDirectCom) ? I2C_IMU_MAG_ADDR : I2C_IMU_GYRO_ADDR;

    // Compute start register based on communication mode
    // Determine register start pointer (points to stable static memory)
    const uint8_t *regPtr = getImuRegPtr(sensor);

    //Switch to bank 0 if needed
    if (!(sensor == IMU_SENSOR_MAG && imuMagDirectCom)) {
        imuSetUsrBank(0);
    }

    // Issue I2C read: uses regPtr which remains valid
    putsI2C1(addr, (uint8_t*)regPtr, 1, buffer, imuSensorLengths[sensor], cb);

    // For direct magnetometer, read ST2 to clear data-ready lock
    if (sensor == IMU_SENSOR_MAG && imuMagDirectCom) {
        static const uint8_t st2 = AK09916_ST2;
        static uint8_t dummy;
        putsI2C1(addr, &st2, 1, &dummy, 1, imuCommCb);
    }
}

/**
 * @brief Common callback for async IMU reads
 *
 * - Checks success
 * - Fixes big-endian to little-endian
 * - Applies axis remapping
 * - Calls update function if provided
 *
 * @param success       I2C read status
 * @param sensor        Sensor type just read
 * @param rawBuffer     Raw int16 buffer (big-endian swapped)
 * @param updateFn      Function to call with fresh data
 */
void imuReadCb(bool success, ImuSensor_t sensor, int16_t *rawBuffer, void (*updateFn)(void)) {
    if (!success) {
        putsUART1Str("Async IMU read error!\r\n");
        return;
    }

    if (sensor != IMU_SENSOR_MAG) {
        // Convert each 16-bit reading from big-endian to native
        for (uint8_t i = 0; i < imuSensorLengths[sensor]/2; i++) {
            rawBuffer[i] = SWAP_BYTES(rawBuffer[i]);
        }
    }

    // Remap axes to robot frame and store in global buffers
    switch (sensor) {
        case IMU_SENSOR_GYRO:
            remapGyroAxes(rawBuffer, rawGyroMeasurements);
            break;
        case IMU_SENSOR_ACCEL:
            remapAccelAxes(rawBuffer, rawAccelMeasurements);
            break;
        case IMU_SENSOR_MAG:
            remapMagAxes(rawBuffer, rawMagMeasurements);
            break;
        case IMU_SENSOR_TEMP:
            rawTempMeasurement = rawBuffer[0];
            break;
    }

    // Notify higher layers (odometry, etc.)
    if (updateFn) {
        updateFn();
    }
}

// Trampolines for async callbacks
static void imuReadGyroCb_trampoline(bool s) {
    imuReadCb(s, IMU_SENSOR_GYRO, localGyroMeasurements, NULL);
}
static void imuReadAccelCb_trampoline(bool s) {
    imuReadCb(s, IMU_SENSOR_ACCEL, localAccelMeasurements, NULL);
}
static void imuReadMagCb_trampoline(bool s) {
    imuReadCb(s, IMU_SENSOR_MAG, localMagMeasurements, NULL);
}
static void imuReadTempCb_trampoline(bool s) {
    imuReadCb(s, IMU_SENSOR_TEMP, &localTempMeasurement, NULL);
}

// Async wrappers calling generic read with trampolines
void imuReadGyro(void) {
    imuGenericRead(IMU_SENSOR_GYRO, (uint8_t*)localGyroMeasurements, imuReadGyroCb_trampoline);
}

void imuReadAccel(void) {
    imuGenericRead(IMU_SENSOR_ACCEL, (uint8_t*)localAccelMeasurements, imuReadAccelCb_trampoline);
}

void imuReadMag(void) {
    imuGenericRead(IMU_SENSOR_MAG, (uint8_t*)localMagMeasurements, imuReadMagCb_trampoline);
}

void imuReadTemp(void) {
    imuGenericRead(IMU_SENSOR_TEMP, (uint8_t*)&localTempMeasurement, imuReadTempCb_trampoline);
}

/**
 * @brief Generic synchronous read helper
 *
 * Performs a blocking read for the given sensor, applies endianness,
 * remapping, and optional scaling.
 *
 * @param sensor    Sensor type to read
 * @param rawBuffer int16 buffer for raw readings
 * @param scaled    Optional float buffer for scaled values
 * @param remap     Whether to remap axes
 * @return true on success
 */
bool imuGenericReadSync(ImuSensor_t sensor, int16_t *rawBuffer, float *scaled, bool remap) {
    uint8_t addr = (sensor == IMU_SENSOR_MAG && imuMagDirectCom) ? I2C_IMU_MAG_ADDR : I2C_IMU_GYRO_ADDR;

    // Compute start register based on communication mode
    // Determine register start pointer (points to stable static memory)
    const uint8_t *regPtr = getImuRegPtr(sensor);
    
    //Switch to bank 0 if needed
    if (!(sensor == IMU_SENSOR_MAG && imuMagDirectCom)) {
        if(!imuSetUsrBankSync(0)) {
            return false;
        }
    }
    
    // Blocking I2C read
    if (!putsI2C1Sync(addr, regPtr, 1, (uint8_t*) rawBuffer, imuSensorLengths[sensor])) {
        return false;
    }
    
    // For direct magnetometer, read ST2 to clear data-ready lock
    if (sensor == IMU_SENSOR_MAG && imuMagDirectCom) {
        static const uint8_t st2 = AK09916_ST2;
        static uint8_t dummy;
        
        if(!putsI2C1Sync(addr, &st2, 1, &dummy, 1)) {
            return false;
        }
    }

    // Endianness fix
    if (sensor != IMU_SENSOR_MAG) {
        for (uint8_t i = 0; i < imuSensorLengths[sensor]/2; i++) {
            rawBuffer[i] = SWAP_BYTES(rawBuffer[i]);
        }
    }

    // Remap axes if requested
    if (remap) {
        switch (sensor) {
            case IMU_SENSOR_GYRO:
                remapGyroAxes(rawBuffer, rawBuffer);
                break;
            case IMU_SENSOR_ACCEL:
                remapAccelAxes(rawBuffer, rawBuffer);
                break;
            case IMU_SENSOR_MAG:
                remapMagAxes(rawBuffer, rawBuffer);
                break;
            default:
                break;
        }
    }

    // Scale if buffer provided
    if (scaled) {
        switch (sensor) {
            case IMU_SENSOR_GYRO:
                imuScaleGyroMeasurements(rawBuffer, scaled);
                break;
            case IMU_SENSOR_ACCEL:
                imuCalibrateAccelMeasurements(rawBuffer, scaled);
                imuScaleAccelMeasurementsFloat(scaled, scaled);
                break;
            case IMU_SENSOR_MAG:
                imuCalibrateMagMeasurements(rawBuffer, scaled);
                imuScaleMagMeasurementsFloat(scaled, scaled);
                break;
            default:
                break;
        }
    }

    return true;
}

bool imuReadGyroSync(int16_t raw[3], float scaled[3], bool remap) {
    return imuGenericReadSync(IMU_SENSOR_GYRO, raw, scaled, remap);
}

bool imuReadAccelSync(int16_t raw[3], float scaled[3], bool remap) {
    return imuGenericReadSync(IMU_SENSOR_ACCEL, raw, scaled, remap);
}

bool imuReadMagSync(int16_t raw[3], float scaled[3], bool remap) {
    return imuGenericReadSync(IMU_SENSOR_MAG, raw, scaled, remap);
}

bool imuReadTempSync(int16_t *raw, float *scaled, bool remap) {
    return imuGenericReadSync(IMU_SENSOR_TEMP, raw, scaled, remap);
}

/*
 * Get FIFO datasets
 */

volatile float rawFifoGyroMeasurements[3];
volatile float rawFifoAccelMeasurements[3];
volatile float rawFifoMagMeasurements[3];
volatile float rawFifoTempMeasurement;

// Local variables that are "unstable".
volatile float localFifoGyroMeasurements[3];
volatile float localFifoAccelMeasurements[3];
volatile float localFifoMagMeasurements[3];
volatile float localFifoTempMeasurement;

#define MAX_DATASET_BYTES   (6+6+9+2)  // accel+gyro+mag+temp
#define MAX_READ_BYTES      105

static volatile bool fifoAligned = false;

static volatile uint16_t fifoCount = 0;
static volatile uint16_t bytesRead = 0;

static uint8_t  fifoData[MAX_READ_BYTES + MAX_DATASET_BYTES];

static inline int16_t read_be16(const uint8_t *ptr) {
    // combine into unsigned, then reinterpret the top bit as sign
    return (int16_t)(((uint16_t)ptr[0] << 8) | (uint16_t)ptr[1]);
}

static inline int16_t read_le16(const uint8_t *ptr) {
    // combine into unsigned, then reinterpret the top bit as sign
    return (int16_t)(((uint16_t)ptr[1] << 8) | (uint16_t)ptr[0]);
}

static void imuProcessFifo(uint16_t bytes) {
    // how many complete samples
    uint16_t numSamples  = bytes / fifoBytesPerDataset;
    uint16_t numMagSamples = 0;

    int32_t gyroSum[3] = {0}, accelSum[3] = {0}, magSum[3] = {0}, tempSum = 0;
    uint16_t offset = 0;

    for (uint16_t i = 0; i < numSamples; i++) {
        if (fifoConfig.accel) {
            for (int j = 0; j < 3; j++) {
                accelSum[j] += read_be16(&fifoData[offset]);
                offset += 2;
            }
        }
        if (fifoConfig.gyro) {
            for (int j = 0; j < 3; j++) {
                gyroSum[j] += read_be16(&fifoData[offset]);
                offset += 2;
            }
        }
        if (fifoConfig.temp) {
            tempSum += read_be16(&fifoData[offset]);
            offset += 2;
        }
        if (fifoConfig.mag) {
            /*
            uint16_t iter = 0;
            if (iter++ % 10 == 0) {
                for (uint8_t k = 0; k < 9; k++) {
                    char buf[20];
                    snprintf(buf, sizeof(buf), "%hu: 0x%02hhX\t", k, fifoData[offset + k]);
                    putsUART1Str(buf);
                }
                putsUART1Str("\r\n");
                
                iter = 0;
            }
            */
            
            uint8_t magSt1 = fifoData[offset++];
            int16_t magData[3];
            
            for (int j = 0; j < 3; j++) {
                magData[j] = read_le16(&fifoData[offset]);
                offset += 2;
            }
            
            offset += 1; // skip unknown register content
            uint8_t magSt2 = fifoData[offset++];
            
            bool magSampleCorrect = (magSt1 & 0x01) && !(magSt2 & 0x08);
                    
            if (magSampleCorrect) {
                for (int j = 0; j < 3; j++) {
                    magSum[j] += magData[j];
                }
                numMagSamples++;
            }
        }
    }

    // TODO: Set in initIMU
    const float odr = 562.5f;
    const float dtPerSample = 1.0f / odr;
    
    float dtTotal = numSamples * dtPerSample;
    
    // Float averaging
    if (numSamples > 0) {
        if (fifoConfig.gyro) {
            for (int j = 0; j < 3; j++) {
                localFifoGyroMeasurements[j] = (float) gyroSum[j] / (float) numSamples;
            }
            remapGyroAxesFloat(localFifoGyroMeasurements, rawFifoGyroMeasurements);
            
            //odometryIMUGyroUpdate(numSamples, dtPerSample, dtTotal);
        }
        if (fifoConfig.accel) {
            for (int j = 0; j < 3; j++) {
                localFifoAccelMeasurements[j] =  (float) accelSum[j] / (float) numSamples;
            }
            remapAccelAxesFloat(localFifoAccelMeasurements, rawFifoAccelMeasurements);
            
            //odometryIMUAccelUpdate(numSamples, dtPerSample, dtTotal);
        }
        if (fifoConfig.mag && numMagSamples > 0) {
            for (int j = 0; j < 3; j++) {
                localFifoMagMeasurements[j] = (float) magSum[j] / (float) numMagSamples;
            }
            remapMagAxesFloat(localFifoMagMeasurements, rawFifoMagMeasurements);
            
            //odometryIMUMagUpdate(dtTotal);
        }
        if (fifoConfig.temp) {
            localFifoTempMeasurement = (float) tempSum / (float) numSamples;
            
            rawFifoTempMeasurement = localFifoTempMeasurement;
        }
    }
}


static void imuReadFifoCb(bool success) {
    if (!success) {
        putsUART1Str("FIFO read failed\r\n");
        return;
    }

    imuProcessFifo(bytesRead);
}

// FIFO read callback
static void imuReadFifoCountCb(bool success) {
    if (!success) {
        putsUART1Str("FIFO count read failed\r\n");
        return;
    }

    fifoCount = SWAP_BYTES(fifoCount);
    
    static uint8_t fifoDataReg[] = {ICM20948_FIFO_R_W};
    
     // ---- ALIGN ONCE ----
    if (!fifoAligned) {
        // compute how many "junk" bytes to throw away
        uint16_t startOffset = fifoCount % fifoBytesPerDataset;
        
        if (startOffset > 0) {
            // do a dummy read of exactly startOffset bytes
            static uint8_t dummyBuf[MAX_DATASET_BYTES];
            imuSetUsrBank(0);
            // if startOffset > sizeof(dummyBuf) you can loop or malloc, but typically it's small
            imuRead(fifoDataReg, 1, dummyBuf, startOffset, NULL);
        }
        
        fifoAligned = true;
        return;  // wait until next timer tick to do real processing
    }
    // ---- END ALIGN ----
    
    // enforce byte limit
    uint16_t bytesToRead  = bytesToRead = fifoCount - (fifoCount % fifoBytesPerDataset);

    if (bytesToRead > MAX_READ_BYTES)
        bytesToRead = MAX_READ_BYTES - (MAX_READ_BYTES % fifoBytesPerDataset);

    if (bytesToRead == 0)
        return;  // nothing new we can process this cycle

    //char buf[20];
    //snprintf(buf, sizeof(buf), "cnt: %u\tread: %u\t", fifoCount, bytesToRead);
    //putsUART1Str(buf);
    
    imuSetUsrBank(0);
    imuRead(fifoDataReg, 1, fifoData, bytesToRead, imuReadFifoCb);
    
    bytesRead = bytesToRead;
}

// Trigger FIFO reading from a timer interrupt
void imuReadFifo(void) {
    if (!fifoAligned) {
        imuFifoReset();
        __delay_ms(1);
    }
    
    static uint8_t fifoCountReg[] = {ICM20948_FIFO_COUNTH};

    imuSetUsrBank(0);
    imuRead(fifoCountReg, 1, (uint8_t*)&fifoCount, 2, imuReadFifoCountCb);
}

bool imuReadFifoSync(void) {
    const uint8_t fifoCountReg = ICM20948_FIFO_COUNTH;
    const uint8_t fifoDataReg = ICM20948_FIFO_R_W;

    if (!fifoAligned) {
        imuFifoReset();
        __delay_ms(1);
    }
    
    IMU_CHECK_BANK_SYNC(0);

    if (!imuReadSync(fifoCountReg, (uint8_t*)&fifoCount, 2)) {
        return false;
    }

    fifoCount = SWAP_BYTES(fifoCount);

     // ---- ALIGN ONCE ----
    if (!fifoAligned) {
        // compute how many "junk" bytes to throw away
        uint16_t startOffset = fifoCount % fifoBytesPerDataset;
        
        if (startOffset > 0) {
            // do a dummy read of exactly startOffset bytes
            static uint8_t dummyBuf[MAX_DATASET_BYTES];
            IMU_CHECK_BANK_SYNC(0);
            // if startOffset > sizeof(dummyBuf) you can loop or malloc, but typically it's small
            imuReadSync(fifoDataReg, dummyBuf, startOffset);
        }
        
        fifoAligned = true;
        return false;  // wait until next timer tick to do real processing
    }

    uint16_t bytesToRead = fifoCount - (fifoCount % fifoBytesPerDataset);
    if (bytesToRead > MAX_READ_BYTES)
        bytesToRead = MAX_READ_BYTES - (MAX_READ_BYTES % fifoBytesPerDataset);

    if (bytesToRead == 0)
        return true;  // nothing new we can process this cycle

    IMU_CHECK_BANK_SYNC(0);

    if (!imuReadSync(fifoDataReg, fifoData, bytesToRead)) {
        return false;
    }

    imuProcessFifo(bytesToRead);
    
    return true;
}

/*
 * Tools
 */

static inline float wrapPi(float angle)
{
    if      (angle >=  M_PI) angle -= 2.0f*M_PI;
    else if (angle <  -M_PI) angle += 2.0f*M_PI;
    
    return angle;
}

float magnetometerToHeading(float scaledMag[3]) {
    float headingRadians = -atan2f(scaledMag[X], scaledMag[Y]);

    headingRadians = wrapPi(headingRadians);

    return headingRadians;
}

float magnetometerToTiltCompensatedHeading(float scaledMag[3], float pitchRad, float rollRad)
{
    // precompute sines & cosines
    float pitchSin = sinf(pitchRad);
    float pitchCos = cosf(pitchRad);
    float rollSin  = sinf(rollRad);
    float rollCos  = cosf(rollRad);
    
    // "east" component (x-axis of horizontal plane)
    float xHorizontal = scaledMag[Y] * pitchCos
                      + scaledMag[Z] * pitchSin;

    // "north" component (y-axis of horizontal plane)
    float yHorizontal = scaledMag[X] * rollCos
                      - scaledMag[Y] * rollSin * pitchSin
                      + scaledMag[Z] * rollSin * pitchCos;
    
    // heading = angle from north toward east
    float headingRadians = -atan2f(yHorizontal, xHorizontal);
    
    // wrap into [-pi, +pi)
    headingRadians = wrapPi(headingRadians);

    return headingRadians;
}


void imuGetAccelCalibrationData(void) {
    putsUART1StrSync("--- Accelerometer Calibration ---\r\n");
    putsUART1StrSync("Collecting a sample every 250ms.\r\n");
    putsUART1StrSync("Rotate the device to different positions and hold still.\r\n");
    putsUART1StrSync("Copy values for calibration for which you know only gravity acted on the accelerometer.\r\n");
    putsUART1StrSync("Endless loop, shutdown when finished.\r\n");
    
    __delay_ms(1000);
    
    int16_t rawAccelMeasurements[3];
    char measurementStr[40];

    uint32_t i = 0;
    
    while (true) {
        bool status = imuReadAccelSync(rawAccelMeasurements, NULL, true);
        if (!status) {
            continue;
        }
    
        i++;
    
        snprintf(measurementStr, sizeof(measurementStr), "%lu, %d, %d, %d\r\n", i, rawAccelMeasurements[0], rawAccelMeasurements[1], rawAccelMeasurements[2]);
        
        //float scaledAccelMeasurements[3];
        //imuCalibrateAccelMeasurements(rawAccelMeasurements, scaledAccelMeasurements);<
        //snprintf(measurementStr, sizeof(measurementStr), "%lu, %.2f, %.2f, %.2f\r\n", i, scaledAccelMeasurements[0], scaledAccelMeasurements[1], scaledAccelMeasurements[2]);
        putsUART1StrSync(measurementStr);
        
        __delay_ms(250);
    }
}

void imuGetMagCalibrationData(void) {
    putsUART1StrSync("--- Magnetometer Calibration ---\r\n");
    putsUART1StrSync("Collecting samples -> rotate the device\r\n");
    
    __delay_ms(1000);
    
    const uint16_t numSamples = 4096;
    
    int16_t rawMagMeasurements[3];
    char measurementStr[30];

    uint16_t i = 0;
    while (i < numSamples) {
        bool status = imuReadMagSync(rawMagMeasurements, NULL, true);
        if (!status) {
            continue;
        }
        i++;
        
        snprintf(measurementStr, sizeof(measurementStr), "%d, %d, %d, %d\r\n", i, rawMagMeasurements[0], rawMagMeasurements[1], rawMagMeasurements[2]);
        putsUART1StrSync(measurementStr);
        
        __delay_ms(15);
    }
    
    putsUART1StrSync("Samples collected. Use provided values for calibration.\r\n");
}