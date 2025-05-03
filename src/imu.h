#ifndef IMUFIFO_H
#define	IMUFIFO_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

extern volatile int16_t rawGyroMeasurements[3];
extern volatile int16_t rawAccelMeasurements[3];
extern volatile int16_t rawMagMeasurements[3];
extern volatile int16_t rawTempMeasurement;

extern volatile float rawFifoGyroMeasurements[3];
extern volatile float rawFifoAccelMeasurements[3];
extern volatile float rawFifoMagMeasurements[3];
extern volatile float rawFifoTempMeasurement;

// Byte-swap macro to swap big-endian to little-endian format
#define SWAP_BYTES(x)  ((uint16_t)((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8)))
#define HIGH_BYTE(x) ((uint8_t)((x) >> 8))
#define LOW_BYTE(x)  ((uint8_t)((x) & 0xFF))

// Unified IMU sensor enumeration
// Select which sensor to read: gyro, accel, mag, or temp
typedef enum {
    IMU_SENSOR_GYRO = 0,   // Gyroscope output
    IMU_SENSOR_ACCEL,      // Accelerometer output
    IMU_SENSOR_MAG,        // Magnetometer output
    IMU_SENSOR_TEMP        // Temperature output
} ImuSensor_t;


typedef enum {
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
} GyroRange_t;

typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
} AccelRange_t;

typedef enum {
    MAG_MODE_OFF = 0b00000,
    MAG_MODE_SINGLE = 0b00001,
    MAG_MODE_10HZ = 0b00010,
    MAG_MODE_20HZ = 0b00100,
    MAG_MODE_50HZ = 0b00110,
    MAG_MODE_100HZ = 0b01000,
    MAG_MODE_SELF_TEST = 0b10000,
} MagMode_t;

typedef enum {
    TEMP_ON = 0,
    TEMP_OFF
} TempMode_t;

typedef struct {
    bool gyro;    /**< true to collect gyroscope data */
    bool accel;   /**< true to collect accelerometer data */
    bool mag;     /**< true to collect magnetometer data */
    bool temp;    /**< true to collect temperature data */
} FifoConfig_t;

#define I2C_IMU_GYRO_ADDR   0x69
#define I2C_IMU_MAG_ADDR 0x0C 

#define ICM20948_ID 0xEA     // The chip ID for the Gyro / Accelerometer
#define AK09916_ID  0x09     // The chip ID for the magnetometer

#define ICM20948_LSB_PER_C 333.87 // temp data LSB value (fixed)
#define AK09916_UT_PER_LSB 0.15 // mag data LSB value (fixed)

// ICM-20948
// Sources: 
// https://github.com/drcpattison/ICM-20948/blob/da9a18efca7b1ee3d5e91911240855b1c6392216/src/ICM20948.h#L31C1-L160C34
// https://github.com/adafruit/Adafruit_ICM20X/blob/5089e9be159ba8ef654e4f0c607d14dc77069461/Adafruit_ICM20948.h#L26

//Magnetometer Registers

#define AK09916_WHO_AM_I 0x01  // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL2    0x31  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL3    0x32  // Normal (0), Reset (1)

// REGISTER FOR ALL USER BANKS
#define ICM20948_REG_BANK_SEL	   0x7F

// USER BANK 0 REGISTER MAP
#define ICM20948_WHO_AM_I           0x00 // Should return 0xEA
#define ICM20948_USER_CTRL          0x03 // Bit 7 enable DMP, bit 3 reset DMP
#define ICM20948_LP_CONFIG		    0x05 
#define ICM20948_PWR_MGMT_1         0x06 // Device defaults to SLEEP mode
#define ICM20948_PWR_MGMT_2         0x07
#define ICM20948_INT_PIN_CFG        0x0F
#define ICM20948_INT_ENABLE         0x10
#define ICM20948_INT_ENABLE_1	    0x11 
#define ICM20948_INT_ENABLE_2	    0x12 
#define ICM20948_INT_ENABLE_3	    0x13 
#define ICM20948_I2C_MST_STATUS     0x17
#define ICM20948_INT_STATUS         0x19
#define ICM20948_INT_STATUS_1	    0x1A
#define ICM20948_INT_STATUS_2	    0x1B
#define ICM20948_INT_STATUS_3	    0x1C
#define ICM20948_DELAY_TIMEH		0x28
#define ICM20948_DELAY_TIMEL		0x29
#define ICM20948_ACCEL_XOUT_H       0x2D
#define ICM20948_ACCEL_XOUT_L       0x2E
#define ICM20948_ACCEL_YOUT_H       0x2F
#define ICM20948_ACCEL_YOUT_L       0x30
#define ICM20948_ACCEL_ZOUT_H       0x31
#define ICM20948_ACCEL_ZOUT_L       0x32
#define ICM20948_GYRO_XOUT_H        0x33
#define ICM20948_GYRO_XOUT_L        0x34
#define ICM20948_GYRO_YOUT_H        0x35
#define ICM20948_GYRO_YOUT_L        0x36
#define ICM20948_GYRO_ZOUT_H        0x37
#define ICM20948_GYRO_ZOUT_L        0x38
#define ICM20948_TEMP_OUT_H         0x39
#define ICM20948_TEMP_OUT_L         0x3A
#define ICM20948_EXT_SENS_DATA_00   0x3B
#define ICM20948_EXT_SENS_DATA_01   0x3C
#define ICM20948_EXT_SENS_DATA_02   0x3D
#define ICM20948_EXT_SENS_DATA_03   0x3E
#define ICM20948_EXT_SENS_DATA_04   0x3F
#define ICM20948_EXT_SENS_DATA_05   0x40
#define ICM20948_EXT_SENS_DATA_06   0x41
#define ICM20948_EXT_SENS_DATA_07   0x42
#define ICM20948_EXT_SENS_DATA_08   0x43
#define ICM20948_EXT_SENS_DATA_09   0x44
#define ICM20948_EXT_SENS_DATA_10   0x45
#define ICM20948_EXT_SENS_DATA_11   0x46
#define ICM20948_EXT_SENS_DATA_12   0x47
#define ICM20948_EXT_SENS_DATA_13   0x48
#define ICM20948_EXT_SENS_DATA_14   0x49
#define ICM20948_EXT_SENS_DATA_15   0x4A
#define ICM20948_EXT_SENS_DATA_16   0x4B
#define ICM20948_EXT_SENS_DATA_17   0x4C
#define ICM20948_EXT_SENS_DATA_18   0x4D
#define ICM20948_EXT_SENS_DATA_19   0x4E
#define ICM20948_EXT_SENS_DATA_20   0x4F
#define ICM20948_EXT_SENS_DATA_21   0x50
#define ICM20948_EXT_SENS_DATA_22   0x51
#define ICM20948_EXT_SENS_DATA_23   0x52
#define ICM20948_FIFO_EN_1          0x66
#define ICM20948_FIFO_EN_2          0x67
#define ICM20948_FIFO_RST           0x68
#define ICM20948_FIFO_MODE	        0x69
#define ICM20948_FIFO_COUNTH        0x70
#define ICM20948_FIFO_COUNTL        0x71
#define ICM20948_FIFO_R_W           0x72
#define ICM20948_DATA_RDY_STATUS	0x74
#define ICM20948_FIFO_CFG	        0x76

// USER BANK 1 REGISTER MAP
#define ICM20948_SELF_TEST_X_GYRO  			0x02
#define ICM20948_SELF_TEST_Y_GYRO  			0x03
#define ICM20948_SELF_TEST_Z_GYRO  			0x04
#define ICM20948_SELF_TEST_X_ACCEL 			0x0E
#define ICM20948_SELF_TEST_Y_ACCEL 			0x0F
#define ICM20948_SELF_TEST_Z_ACCEL 			0x10
#define ICM20948_XA_OFFSET_H       			0x14
#define ICM20948_XA_OFFSET_L       			0x15
#define ICM20948_YA_OFFSET_H       			0x17
#define ICM20948_YA_OFFSET_L       			0x18
#define ICM20948_ZA_OFFSET_H       			0x1A
#define ICM20948_ZA_OFFSET_L       			0x1B
#define ICM20948_TIMEBASE_CORRECTION_PLL	0x28

// USER BANK 2 REGISTER MAP
#define ICM20948_GYRO_SMPLRT_DIV        	0x00
#define ICM20948_GYRO_CONFIG_1      		0x01
#define ICM20948_GYRO_CONFIG_2      		0x02
#define ICM20948_XG_OFFSET_H       		    0x03  // User-defined trim values for gyroscope
#define ICM20948_XG_OFFSET_L       		    0x04
#define ICM20948_YG_OFFSET_H       		    0x05
#define ICM20948_YG_OFFSET_L       		    0x06
#define ICM20948_ZG_OFFSET_H       		    0x07
#define ICM20948_ZG_OFFSET_L       		    0x08
#define ICM20948_ODR_ALIGN_EN			    0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1     	0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2     	0x11
#define ICM20948_ACCEL_INTEL_CTRL		    0x12
#define ICM20948_ACCEL_WOM_THR			    0x13
#define ICM20948_ACCEL_CONFIG      		    0x14
#define ICM20948_ACCEL_CONFIG_2     		0x15
#define ICM20948_FSYNC_CONFIG			    0x52
#define ICM20948_TEMP_CONFIG				0x53
#define ICM20948_MOD_CTRL_USR			    0x54

// USER BANK 3 REGISTER MAP
#define ICM20948_I2C_MST_ODR_CONFIG		    0x00
#define ICM20948_I2C_MST_CTRL       		0x01
#define ICM20948_I2C_MST_DELAY_CTRL 		0x02
#define ICM20948_I2C_SLV0_ADDR      		0x03
#define ICM20948_I2C_SLV0_REG       		0x04
#define ICM20948_I2C_SLV0_CTRL      		0x05
#define ICM20948_I2C_SLV0_DO        		0x06
#define ICM20948_I2C_SLV1_ADDR      		0x07
#define ICM20948_I2C_SLV1_REG       		0x08
#define ICM20948_I2C_SLV1_CTRL      		0x09
#define ICM20948_I2C_SLV1_DO        		0x0A
#define ICM20948_I2C_SLV2_ADDR      		0x0B
#define ICM20948_I2C_SLV2_REG       		0x0C
#define ICM20948_I2C_SLV2_CTRL      		0x0D
#define ICM20948_I2C_SLV2_DO        		0x0E
#define ICM20948_I2C_SLV3_ADDR      		0x0F
#define ICM20948_I2C_SLV3_REG       		0x10
#define ICM20948_I2C_SLV3_CTRL      		0x11
#define ICM20948_I2C_SLV3_DO        		0x12
#define ICM20948_I2C_SLV4_ADDR      		0x13
#define ICM20948_I2C_SLV4_REG       		0x14
#define ICM20948_I2C_SLV4_CTRL      		0x15
#define ICM20948_I2C_SLV4_DO        		0x16
#define ICM20948_I2C_SLV4_DI        		0x17

/**
 * @brief General async callback to alert something went wrong.
 */
void imuCommCb(bool success);

/**
 * @brief Switches the user bank to the specified bank.
 */
void imuSetUsrBank(uint8_t bank);

/**
 * @brief Switches the user bank synchronously to the specified bank.
 */
bool imuSetUsrBankSync(uint8_t bank);

/**
 * @brief Writes data asynchronously to the IMU over I2C.
 */
void imuWrite(uint8_t *wBuf, uint16_t wLen, I2CCallback_t cb);

/**
 * @brief Writes a single byte synchronously to the IMU.
 */
bool imuWrite8Sync(uint8_t reg, uint8_t val);

/**
 * @brief Writes two bytes synchronously to the IMU.
 */
bool imuWrite16Sync(uint8_t reg, uint16_t val, bool littleEndian);

/**
 * @brief Writes a buffer synchronously to the IMU.
 */
bool imuWriteBufSync(uint8_t *wBuf, uint16_t len);

/**
 * @brief Reads data asynchronously from the IMU over I2C.
 */
void imuRead(uint8_t *wBuf, uint16_t wLen, uint8_t *rBuf, uint16_t rLen, I2CCallback_t cb);

/**
 * @brief Reads data synchronously from the IMU.
 */
bool imuReadSync(uint8_t reg, uint8_t *val, uint16_t len);

/**
 * @brief Sets or clears a bit in an IMU register synchronously.
 */
bool imuSetBitSync(uint8_t reg, uint8_t bit, bool state);

/**
 * @brief Writes a byte synchronously to the magnetometer.
 */
bool imuMagWriteSync(uint8_t reg, uint8_t val, bool directCom);

/**
 * @brief Reads data synchronously from the magnetometer.
 */
bool imuMagReadSync(uint8_t reg, uint8_t *val, uint16_t len, bool directCom);

/**
 * @brief Sets or clears a bit in a magnetometer register synchronously.
 */
bool imuMagSetBitSync(uint8_t reg, uint8_t bit, bool state, bool directCom);

/**
 * @brief Resets the IMU synchronously.
 */
bool imuResetSync(void);

/**
 * @brief Resets the IMU's I2C master synchronously.
 */
bool imuI2CMasterResetSync(void);

/**
 * @brief Resets the magnetometer synchronously.
 */
bool imuMagResetSync(bool directCom);

/**
 * @brief Resets the IMU FIFO asynchronously.
 */
void imuFifoReset(void);

/**
 * @brief Resets the IMU FIFO synchronously.
 */
bool imuFifoResetSync(void);

/**
 * @brief Configures magnetometer data transfer via FIFO.
 */
bool imuSetupMagDataTransfer(uint8_t reg, uint8_t len, bool swap, bool oddAddrFirst);

/**
 * @brief Sets up the IMU with the specified ranges, modes, and FIFO config.
 */
bool imuSetup(GyroRange_t gyroRange,
              AccelRange_t accelRange,
              MagMode_t magMode,
              TempMode_t tempMode,
              FifoConfig_t fifoCfg);

/**
 * @brief Calibrates the gyroscope offsets.
 */
bool imuCalibrateGyro(void);

/**
 * @brief Calibrates the accelerometer offsets.
 */
bool imuCalibrateAccel(void);

/**
 * @brief Scales raw float gyro measurements into degrees per second.
 */
void imuScaleGyroMeasurementsFloat(float rawGyro[3], float scaledGyro[3]);

/**
 * @brief Scales a single raw float gyro measurement into degrees per second.
 */
void imuScaleGyroMeasurementFloat(float *rawGyro, float *scaledGyro);

/**
 * @brief Scales raw gyro measurements into degrees per second.
 */
void imuScaleGyroMeasurements(int16_t rawGyro[3], float scaledGyro[3]);

/**
 * @brief Scales a single raw gyro measurement into degrees per second.
 */
void imuScaleGyroMeasurement(int16_t *rawGyro, float *scaledGyro);

/**
 * @brief Scales raw float accelerometer measurements into g.
 */
void imuScaleAccelMeasurementsFloat(float rawAccel[3], float scaledAccel[3]);

/**
 * @brief Scales a single raw float accelerometer measurement into g.
 */
void imuScaleAccelMeasurementFloat(float *rawAccel, float *scaledAccel);

/**
 * @brief Scales raw accelerometer measurements into g.
 */
void imuScaleAccelMeasurements(int16_t rawAccel[3], float scaledAccel[3]);

/**
 * @brief Scales a single raw accelerometer measurement into g.
 */
void imuScaleAccelMeasurement(int16_t *rawAccel, float *scaledAccel);

/**
 * @brief Scales raw float magnetometer measurements into microtesla.
 */
void imuScaleMagMeasurementsFloat(float rawMag[3], float scaledMag[3]);

/**
 * @brief Scales a single raw float magnetometer measurement into microtesla.
 */
void imuScaleMagMeasurementFloat(float *rawMag, float *scaledMag);

/**
 * @brief Scales raw magnetometer measurements into microtesla.
 */
void imuScaleMagMeasurements(int16_t rawMag[3], float scaledMag[3]);

/**
 * @brief Scales a single raw magnetometer measurement into microtesla.
 */
void imuScaleMagMeasurement(int16_t *rawMag, float *scaledMag);

/**
 * @brief Scales raw float temperature measurement into degrees Celsius.
 */
void imuScaleTempMeasurementsFloat(float *rawTemp, float *scaledTemp);

/**
 * @brief Scales raw temperature measurement into degrees Celsius.
 */
void imuScaleTempMeasurements(int16_t *rawTemp, float *scaledTemp);

/**
 * @brief Calibrates raw accelerometer measurements using bias and scale.
 */
void imuCalibrateAccelMeasurementsFloat(float *rawAccel, float *scaledAccel);

/**
 * @brief Calibrates raw accelerometer measurements using bias and scale.
 */
void imuCalibrateAccelMeasurements(int16_t *rawAccel, float *scaledAccel);


/**
 * @brief Calibrates raw float magnetometer measurements using bias and scale.
 */
void imuCalibrateMagMeasurementsFloat(float *rawMag, float *scaledMag);

/**
 * @brief Calibrates raw magnetometer measurements using bias and scale.
 */
void imuCalibrateMagMeasurements(int16_t *rawMag, float *scaledMag);

/**
 * @brief Reads sensor data asynchronously.
 */
void imuGenericRead(ImuSensor_t sensor, uint8_t *buffer, I2CCallback_t cb);

/**
 * @brief Common callback for async IMU reads.
 */
void imuReadCb(bool success, ImuSensor_t sensor, int16_t *rawBuffer, void (*updateFn)(void));

/**
 * @brief Initiates asynchronous gyroscope read.
 */
void imuReadGyro(void);

/**
 * @brief Initiates asynchronous accelerometer read.
 */
void imuReadAccel(void);

/**
 * @brief Initiates asynchronous magnetometer read.
 */
void imuReadMag(void);

/**
 * @brief Initiates asynchronous temperature read.
 */
void imuReadTemp(void);

/**
 * @brief Performs a synchronous read for the given sensor.
 */
bool imuGenericReadSync(ImuSensor_t sensor, int16_t *rawBuffer, float *scaled, bool remap);

/**
 * @brief Performs a synchronous gyroscope read.
 */
bool imuReadGyroSync(int16_t raw[3], float scaled[3], bool remap);

/**
 * @brief Performs a synchronous accelerometer read.
 */
bool imuReadAccelSync(int16_t raw[3], float scaled[3], bool remap);

/**
 * @brief Performs a synchronous magnetometer read.
 */
bool imuReadMagSync(int16_t raw[3], float scaled[3], bool remap);

/**
 * @brief Performs a synchronous temperature read.
 */
bool imuReadTempSync(int16_t *raw, float *scaled, bool remap);


void imuReadFifo(void);

bool imuReadFifoSync(void);

/**
 * @brief Converts scaled magnetometer data to heading in radians.
 */
float magnetometerToHeading(float scaledMag[3]);

/**
 * @brief Converts scaled magnetometer data to tilt?compensated heading in radians.
 */
float magnetometerToTiltCompensatedHeading(float scaledMag[3],
                                           float pitchRad,
                                           float rollRad);

/**
 * @brief Streams raw accelerometer data for calibration purposes.
 */
void imuGetAccelCalibrationData(void);

/**
 * @brief Streams raw magnetometer data for calibration purposes.
 */
void imuGetMagCalibrationData(void);

#endif	/* IMUFIFO_H */

