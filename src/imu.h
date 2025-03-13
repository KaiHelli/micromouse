#ifndef IMU_H
#define	IMU_H

#include <stdint.h>
#include <stdbool.h>

static int16_t rawGyroMeasurements[3];
static int16_t rawAccelMeasurements[3];

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

#define I2C_IMU_GYRO_ADDR   0x69
#define I2C_IMU_MAG_ADDR 0x0C 

#define ICM20948_ID 0xEA     // The chip ID for the Gyro / Accelerometer
#define AK09916_ID  0x09     // The chip ID for the magnetometer

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
#define ICM20948_PWR_MGMT_1         0x06 // Device defaults to the SLEEP mode
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

void imuSetup(GyroRange_t gyroRange, AccelRange_t accelRange);
bool imuSetUsrBank(uint8_t bank);
void imuReadWhoAmI(void);
void imuReadGyro(void);
void imuReadAccel(void);
void imuScaleGyroMeasurements(const int16_t rawGyro[3], float scaledGyro[3]);
void imuScaleAccelMeasurements(const int16_t rawAccel[3], float scaledAccel[3]);

float dpsToRadps(float dps);

#endif	/* IMU_H */

