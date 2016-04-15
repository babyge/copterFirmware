/*
 * In this file, the i2c routines for the internal imu/eeprom i2c devices are implemented.
 *
 * As a special feature it contains an autoreading mode. In this mode all configured
 * sensors are updated automatically (e.i. only in interrupts, no external function calls
 * necessary).
 *
 * Optimization potential:
 * The autoreading mode is based on the CPAL library. Each sensor reading consists of two
 * parts:
 * 1. sending the internal memory pointer
 * 2. acquiring sensor data
 * These steps are combined in the CPAL_I2C_Read function, but the function blocks further
 * execution until step 1 is finished (approximately 135µs unnecessary wait time @168MHz,
 * 400000Bitrate, 3 Sensors per cycle).
 * These steps might be executed individually by calling CPAL_I2C_Write and subsequent
 * CPAL_I2C_Read with CPAL_OPTION = CPAL_OPT_NO_MEM_ADDR enabled.
 */
#ifndef I2C_H_
#define I2C_H_

#include "cpal_conf.h"
#include "cpal_i2c.h"
#include "accelerometer.h"
#include "gyro.h"
#include "magnetometer.h"
#include "stdcomm.h"

#define I2C_BITRATE 400000

// number of read/write operations which will be performed in autoreading mode
//#define I2C_CYCLE_LENGTH	3
#define I2C_MAX_CYCLE_LENGTH 3
// sensor reading frequency
#define I2C_CYCLE_FREQUENCY 500

typedef enum {
	WRITING = 0, READING = !WRITING
} I2CDataDirection;

I2CDataDirection cycleDirection[I2C_MAX_CYCLE_LENGTH];
CPAL_TransferTypeDef *cycleTransfer[I2C_MAX_CYCLE_LENGTH];
uint8_t cycleNumOfValues[I2C_MAX_CYCLE_LENGTH];
struct {
	// this flag enables/disables the autoreading feature
	FunctionalState autoReading;
	// current position in the auto-reading cycle
	uint8_t autoPosition;
	uint8_t autoCycleLength;
	/*
	 * Flags incicating available sensors
	 */
	FlagStatus accAvailable, gyroAvailable, magAvailable;
	/*
	 * flag is set when a timeout occurs. in this case the cycle must be started
	 * manual by calling i2c_NextReading
	 */
	FlagStatus error;
	uint32_t errorMessageTimer;
	/*
	 * flag indicates new unread data. It must be reset by the application code once
	 * the data has been read
	 */
	FlagStatus newData;
	/*
	 * flag indicates whether a auto-reading cycle is currently running
	 */
	volatile FlagStatus autoReadingRunning;
} internali2c;

#ifndef _MPU6050_
#define _MPU6050_
#define I2CADDRESS_MPU6050		0b11010000
#define MPU6050_CONFIG			0x1A
#define MPU6050_GYRO_CONFIG		0x1B
#define MPU6050_ACCEL_CONFIG	0x1C
#define MPU6050_INT_PIN_CFG		0x37
#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_ACCEL_XOUT_L	0x3C
#define MPU6050_ACCEL_YOUT_H	0x3D
#define MPU6050_ACCEL_YOUT_L	0x3E
#define MPU6050_ACCEL_ZOUT_H	0x3F
#define MPU6050_ACCEL_ZOUT_L	0x40
#define MPU6050_GYRO_XOUT_H		0x43
#define MPU6050_GYRO_XOUT_L		0x44
#define MPU6050_GYRO_YOUT_H		0x45
#define MPU6050_GYRO_YOUT_L		0x46
#define MPU6050_GYRO_ZOUT_H		0x47
#define MPU6050_GYRO_ZOUT_L		0x48
#define MPU6050_PWR_MGMT_1		0x6B
#endif

#ifndef _HMC5883_
#define _HMC5883_
#define I2CADDRESS_HMC5883		0b00111100
#define HMC5883_CONF_REG_A		0x00
#define HMC5883_CONF_REG_B		0x01
#define HMC5883_MODE			0x02
#define HMC5883_XOUT_H			0x03
#define HMC5883_XOUT_L			0x04
#define HMC5883_ZOUT_H			0x05
#define HMC5883_ZOUT_L			0x06
#define HMC5883_YOUT_H			0x07
#define HMC5883_YOUT_L			0x08
#define HMC5883_STATUS			0x09
#define HMC5883_IDENT_1			0x0A
#define HMC5883_IDENT_2			0x0B
#define HMC5883_IDENT_3			0x0C
#endif

// sensors addresses and subaddresses
#define ACC_ADDRESS			I2CADDRESS_MPU6050
#define ACC_SUBADDRESS 		MPU6050_ACCEL_XOUT_H
#define ACC_NUMOFVALUES		6

#define GYRO_ADDRESS 		I2CADDRESS_MPU6050
#define GYRO_SUBADDRESS 	MPU6050_GYRO_XOUT_H
#define GYRO_NUMOFVALUES	6

#define MAG_ADDRESS 		I2CADDRESS_HMC5883
#define MAG_SUBADDRESS 		HMC5883_XOUT_H
#define MAG_NUMOFVALUES		6

// i2c transfer structures for the sensors
CPAL_TransferTypeDef accDataRX;
CPAL_TransferTypeDef gyroDataRX;
CPAL_TransferTypeDef magDataRX;

/*
 * initializes i2c hardware and fills the cycle arrays
 */
void i2c_Init(void);
/*
 * initializes the internal sensors (MPU6050, HMC5883L)
 * After i2c_Init() followed by this function the sensors
 * are fully functional and the raw values are updated with
 * I2C_CYCLE_FREQUENCY
 */
void i2c_InitSensors(void);

/*
 * return value:
 * 1 new data is available
 * 0 no new data
 *
 * The function clears the new-data-flag
 */
uint8_t i2c_NewSensorData(void);

/*
 * checks whether the i2c device is busy
 */
uint8_t i2c_Busy(void);

uint8_t i2c_WriteRegister(uint8_t adr, uint8_t subadr, uint8_t value);
/*
 * enables/disables cyclic autoreading. this feature should be disabled until all
 * register initializations have been performed
 */
void i2c_Autoreading(FunctionalState newstate);
/*
 * internal function, starts the next transfer process. This function must be called by
 * the user callbacks of the cpal library
 */
void i2c_TransferFinished(void);
/*
 * this function is called by the user callback at an i2c error
 */
void i2c_Error(void);
/*
 * this function is called by the user callback at an i2c timeout
 */
void i2c_Timeout(void);
#endif /* I2C_H_ */
