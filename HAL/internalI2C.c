/*
 * i2c.c
 *
 *  Created on: Oct 30, 2013
 *      Author: jan
 */

#include "internalI2C.h"

/*
 * initializes i2c hardware and fills the cycle arrays
 */
void i2c_Init(void) {
	internali2c.autoReading = DISABLE;
	internali2c.error = RESET;
	internali2c.autoReadingRunning = RESET;
//	// accelerometer transfer structur
//	cycleDirection[0] = READING;
//	cycleTransfer[0] = &accDataRX;
//	cycleNumOfValues[0] = ACC_NUMOFVALUES;
//	// gyro transfer structur
//	cycleDirection[1] = READING;
//	cycleTransfer[1] = &gyroDataRX;
//	cycleNumOfValues[1] = GYRO_NUMOFVALUES;
//	// magnetometer transfer structur
//	cycleDirection[2] = READING;
//	cycleTransfer[2] = &magDataRX;
//	cycleNumOfValues[2] = MAG_NUMOFVALUES;

	CPAL_I2C_StructInit(&I2C1_DevStructure);
	I2C1_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C_BITRATE;
	// init i2c device
	if (CPAL_I2C_Init(&I2C1_DevStructure) == CPAL_PASS)
		stdComm_puts("i2c initialization passed\n");
	else
		stdComm_puts("i2c initialization failed\n");

	/*
	 * setup timer 6 and interrupt
	 * (used for timing the cycle update according to I2C_CYCLE_FREQUENCY)
	 */
	//init timer7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitTypeDef timer;
	// prescaler 840 -> 100000 inc/s
	timer.TIM_Prescaler = 839;
	timer.TIM_Period = (100000L / I2C_CYCLE_FREQUENCY) - 1;
	TIM_TimeBaseInit(TIM6, &timer);
	TIM_Cmd(TIM6, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//enable timer interrupt
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}
/*
 * initializes the internal sensors (MPU6050, HMC5883L)
 * After i2c_Init() followed by this function the sensors
 * are fully functional and the raw values are updated with
 * I2C_CYCLE_FREQUENCY
 */
void i2c_InitSensors(void) {
	accDataRX.pbBuffer = accelerometer.rawValues;
	accDataRX.wAddr1 = ACC_ADDRESS;
	accDataRX.wAddr2 = ACC_SUBADDRESS; // | 0x80;
	accDataRX.wNumData = ACC_NUMOFVALUES;

	gyroDataRX.pbBuffer = gyro.rawValues;
	gyroDataRX.wAddr1 = GYRO_ADDRESS;
	gyroDataRX.wAddr2 = GYRO_SUBADDRESS; // | 0x80;
	gyroDataRX.wNumData = GYRO_NUMOFVALUES;

	magDataRX.pbBuffer = magnetometer.rawValues;
	magDataRX.wAddr1 = MAG_ADDRESS;
	magDataRX.wAddr2 = MAG_SUBADDRESS;
	magDataRX.wNumData = MAG_NUMOFVALUES;

	internali2c.autoCycleLength = 0;

	uint8_t error = 0;
	// initialize MPU6050
	// disable sleep mode
	if (i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_PWR_MGMT_1, 0x00))
		error = 1;
//	// digital low pass filter, bandwidth = 260Hz (acc) 256Hz (gyro)
//	i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_CONFIG, 0x00);
	// digital low pass filter, bandwidth = 94Hz (acc) 98Hz (gyro)
	if (i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_CONFIG, 0x02))
		error = 1;
//	// measurement interval +/- 500°/s
//	i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_GYRO_CONFIG, 0x08);
	// measurement interval +/- 2000°/s
	if (i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_GYRO_CONFIG, 0x18))
		error = 1;
	// measurement interval +/-8g
	if (i2c_WriteRegister(I2CADDRESS_MPU6050, MPU6050_ACCEL_CONFIG, 0x10))
		error = 1;
	if (error) {
		log_LogFileEntry("ERROR: couldn't initialize acc/gyro");
		internali2c.accAvailable = RESET;
		internali2c.gyroAvailable = RESET;
	} else {
		log_LogFileEntry("initialized acc/gyro");
		// setup autoreading structure
		// accelerometer transfer structur
		cycleDirection[internali2c.autoCycleLength] = READING;
		cycleTransfer[internali2c.autoCycleLength] = &accDataRX;
		cycleNumOfValues[internali2c.autoCycleLength] = ACC_NUMOFVALUES;
		internali2c.autoCycleLength++;
		// gyro transfer structur
		cycleDirection[internali2c.autoCycleLength] = READING;
		cycleTransfer[internali2c.autoCycleLength] = &gyroDataRX;
		cycleNumOfValues[internali2c.autoCycleLength] = GYRO_NUMOFVALUES;
		internali2c.autoCycleLength++;
		internali2c.accAvailable = SET;
		internali2c.gyroAvailable = SET;
	}

	error = 0;
	// initialize HMC5883
	// 1 sample average, 75 Hz update rate, normal measurement mode
	if (i2c_WriteRegister(I2CADDRESS_HMC5883, HMC5883_CONF_REG_A, 0x18))
		error = 1;
	// +/-0.88 Gauss
	if (i2c_WriteRegister(I2CADDRESS_HMC5883, HMC5883_CONF_REG_B, 0x00))
		error = 1;
	// continuous measurement mode
	if (i2c_WriteRegister(I2CADDRESS_HMC5883, HMC5883_MODE, 0x00))
		error = 1;
	if (error) {
		log_LogFileEntry("ERROR: couldn't initialize magnetometer");
		internali2c.magAvailable = RESET;
	} else {
		log_LogFileEntry("initialized magnetometer");
		// setup autoreading structure
		// magnetometer transfer structur
		cycleDirection[internali2c.autoCycleLength] = READING;
		cycleTransfer[internali2c.autoCycleLength] = &magDataRX;
		cycleNumOfValues[internali2c.autoCycleLength] = MAG_NUMOFVALUES;
		internali2c.autoCycleLength++;
		internali2c.magAvailable = SET;
	}
	i2c_Autoreading(ENABLE);
}

/*
 * return value:
 * 1 new data is available
 * 0 no new data
 */
uint8_t i2c_NewSensorData(void) {
	if (internali2c.newData == SET) {
		internali2c.newData = RESET;
		return 1;
	} else
		return 0;
}
/*
 * checks whether the i2c device is busy
 */
uint8_t i2c_Busy(void) {
	if (I2C1_DevStructure.CPAL_State == CPAL_STATE_READY
			|| I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
		return 0;
	else
		return 1;
}
uint8_t i2c_WriteRegister(uint8_t adr, uint8_t subadr, uint8_t value) {
	internali2c.error = RESET;
	CPAL_TransferTypeDef tx;
	tx.pbBuffer = &value;
	tx.wAddr1 = adr;
	tx.wAddr2 = subadr;
	tx.wNumData = 1;
	while (i2c_Busy())
		;
	I2C1_DevStructure.pCPAL_TransferTx = &tx;
	CPAL_I2C_Write(&I2C1_DevStructure);
	while (i2c_Busy())
		;
	if (I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
		return 1;
	if (internali2c.error == SET)
		return 1;
	return 0;
}
/*
 * enables/disables cyclic autoreading. this feature should be disabled until all
 * register initializations have been performed
 */
void i2c_Autoreading(FunctionalState newstate) {
	internali2c.autoReading = newstate;
}
/*
 * this interrupt starts the auto-reading cycle
 */
void TIM6_DAC_IRQHandler(void) {
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		// check whether auto-reading is enabled and the previous cycle is finished
		if (internali2c.autoReading == ENABLE
				&& internali2c.autoReadingRunning == RESET
				&& internali2c.autoCycleLength > 0) {
			// start next cycle
			internali2c.autoPosition = 0;
			internali2c.autoReadingRunning = SET;
			i2c_TransferFinished();
		}
	}
}
/*
 * internal function, starts the next transfer process. This function must be called by
 * the user callbacks of the cpal library
 */
void i2c_TransferFinished(void) {
	if (internali2c.autoReadingRunning) {
		if (internali2c.autoPosition < internali2c.autoCycleLength) {
			// reset byte count in transfer structur
			cycleTransfer[internali2c.autoPosition]->wNumData =
					cycleNumOfValues[internali2c.autoPosition];
			// load next transfer structur into the device structur and start the transmission
			if (cycleDirection[internali2c.autoPosition] == READING) {
				I2C1_DevStructure.pCPAL_TransferRx =
						cycleTransfer[internali2c.autoPosition];
				if (CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS)
					internali2c.error = 0;
			} else {
				I2C1_DevStructure.pCPAL_TransferTx =
						cycleTransfer[internali2c.autoPosition];
				if (CPAL_I2C_Write(&I2C1_DevStructure) == CPAL_PASS)
					internali2c.error = 0;
			}
			// increase the autoreading position
			internali2c.autoPosition++;
		} else {
			internali2c.autoReadingRunning = RESET;
			internali2c.newData = SET;
		}
	}
}
/*
 * this function is called by the user callback at an i2c error
 */
void i2c_Error(void) {
	internali2c.error = SET;
	internali2c.autoReadingRunning = RESET;
	hott.speak = SPEAK_ERR_DATABUS;
	if (time_TimerElapsed(&internali2c.errorMessageTimer)) {
		time_SetTimer(&internali2c.errorMessageTimer, 500);
		log_LogFileEntry("ERROR: internal I2C");
	}
}
/*
 * this function is called by the user callback at an i2c timeout
 */
void i2c_Timeout(void) {
	i2c_Error();
}

