/*
 * eeprom.c
 *
 *  Created on: Oct 31, 2013
 *      Author: jan
 */

#include "eeprom.h"

/*
 * writes a single byte to the eeprom
 */
void eeprom_WriteByte(uint16_t adr, uint8_t data) {
	// save current state
	FunctionalState state = internali2c.autoReading;
	// disable auto reading and wait for the current cycle to finish
	i2c_Autoreading(DISABLE);
	while (internali2c.autoReadingRunning)
		;
	internali2c.error = RESET;
	// setup transfer structur
	CPAL_TransferTypeDef write;
	write.wAddr1 = EEPROM_ADDRESS;
	write.wAddr2 = adr;
	write.pbBuffer = &data;
	write.wNumData = 1;
	// activate 16-Bit address mode
	I2C1_DevStructure.wCPAL_Options |= CPAL_OPT_16BIT_REG;
	// transfer data
	while (i2c_Busy())
		;
	I2C1_DevStructure.pCPAL_TransferTx = &write;
	CPAL_I2C_Write(&I2C1_DevStructure);
	while (i2c_Busy())
		;
	// deactivate 16-Bit adress mode
	I2C1_DevStructure.wCPAL_Options &= ~CPAL_OPT_16BIT_REG;
	/*
	 * wait for the eeprom to finish the write cycle.
	 * this might be skipped if it can be guaranteed that
	 * there will be always a delay of at least 10ms between
	 * two write operations
	 */
	time_Waitms(10);
	// set old state again
	i2c_Autoreading(state);
}
/*
 * reads a single byte from the eeprom
 */
void eeprom_ReadByte(uint16_t adr, uint8_t *data) {
	// save current state
	FunctionalState state = internali2c.autoReading;
	// disable auto reading and wait for the current cycle to finish
	i2c_Autoreading(DISABLE);
	while (internali2c.autoReadingRunning)
		;
	internali2c.error = RESET;
	// setup transfer structur
	CPAL_TransferTypeDef read;
	read.wAddr1 = EEPROM_ADDRESS;
	read.wAddr2 = adr;
	read.pbBuffer = data;
	read.wNumData = 1;
	// activate 16-Bit address mode
	I2C1_DevStructure.wCPAL_Options |= CPAL_OPT_16BIT_REG;
	// transfer data
	while (i2c_Busy())
		;
	I2C1_DevStructure.pCPAL_TransferRx = &read;
	CPAL_I2C_Read(&I2C1_DevStructure);
	while (i2c_Busy())
		;
	// deactivate 16-Bit adress mode
	I2C1_DevStructure.wCPAL_Options &= ~CPAL_OPT_16BIT_REG;
	// set old state again
	i2c_Autoreading(state);
}
/*
 * writes a page (see EEPROM_PAGE_SIZE) to the eeprom
 */
void eeprom_WritePage(uint16_t adr, uint8_t *data, uint8_t size) {
	// save current state
	FunctionalState state = internali2c.autoReading;
	// disable auto reading and wait for the current cycle to finish
	i2c_Autoreading(DISABLE);
	while (internali2c.autoReadingRunning)
		;
	internali2c.error = RESET;
	// setup transfer structur
	CPAL_TransferTypeDef write;
	write.wAddr1 = EEPROM_ADDRESS;
	write.wAddr2 = adr;
	write.pbBuffer = data;
	if (size > EEPROM_PAGE_SIZE)
		size = EEPROM_PAGE_SIZE;
	write.wNumData = size;
	// activate 16-Bit address mode
	I2C1_DevStructure.wCPAL_Options |= CPAL_OPT_16BIT_REG;
	// transfer data
	while (i2c_Busy())
		;
	I2C1_DevStructure.pCPAL_TransferTx = &write;
	CPAL_I2C_Write(&I2C1_DevStructure);
	while (i2c_Busy())
		;
	// deactivate 16-Bit adress mode
	I2C1_DevStructure.wCPAL_Options &= ~CPAL_OPT_16BIT_REG;
	/*
	 * wait for the eeprom to finish the write cycle.
	 * this might be skipped if it can be guaranteed that
	 * there will be always a delay of at least 10ms between
	 * two write operations
	 */
	time_Waitms(20);
	// set old state again
	i2c_Autoreading(state);
}
/*
 * reads a block of data from the eeprom
 */
void eeprom_ReadBlock(uint16_t adr, uint8_t *data, uint16_t size) {
	// save current state
	FunctionalState state = internali2c.autoReading;
	// disable auto reading and wait for the current cycle to finish
	i2c_Autoreading(DISABLE);
	while (internali2c.autoReadingRunning)
		;
	internali2c.error = RESET;
	// setup transfer structur
	CPAL_TransferTypeDef read;
	read.wAddr1 = EEPROM_ADDRESS;
	read.wAddr2 = adr;
	read.pbBuffer = data;
	read.wNumData = size;
	// activate 16-Bit address mode
	I2C1_DevStructure.wCPAL_Options |= CPAL_OPT_16BIT_REG;
	// transfer data
	while (i2c_Busy())
		;
	I2C1_DevStructure.pCPAL_TransferRx = &read;
	CPAL_I2C_Read(&I2C1_DevStructure);
	while (i2c_Busy())
		;
	// deactivate 16-Bit adress mode
	I2C1_DevStructure.wCPAL_Options &= ~CPAL_OPT_16BIT_REG;
	// set old state again
	i2c_Autoreading(state);
}
/*
 * writes a block of data to the eeprom
 */
void eeprom_WriteBlock(uint16_t adr, uint8_t *data, uint16_t size) {
	// save current state
	FunctionalState state = internali2c.autoReading;
	// disable auto reading and wait for the current cycle to finish
	i2c_Autoreading(DISABLE);
	while (internali2c.autoReadingRunning)
		;
	internali2c.error = RESET;

	uint16_t writtenBytes = 0;

	// check whether address is aligned with page boundary
	if (adr % EEPROM_PAGE_SIZE) {
		// address doesn't start at the beginning of a page
		// -> fill partial page first
		uint8_t page_space = ((int) (adr / EEPROM_PAGE_SIZE) + 1)
				* EEPROM_PAGE_SIZE - adr;
		eeprom_WritePage(adr, data, page_space);
		writtenBytes = page_space;
	}
	while (writtenBytes < size) {
		// current block starts at EEPROM page beginning
		// -> write up to EEPROM_PAGE_SIZE bytes
		uint16_t blocksize = (size - writtenBytes);
		if (blocksize > EEPROM_PAGE_SIZE)
			blocksize = EEPROM_PAGE_SIZE;
		eeprom_WritePage(adr + writtenBytes, &data[writtenBytes], blocksize);
		writtenBytes += blocksize;
	}

	// check for error
	if (internali2c.error == SET) {
		internali2c.error = RESET;
#ifdef DEBUG_UART
		stdComm_puts("I2C error eeprom\n");
#endif
		log_LogFileEntry("ERROR: I2C error while writing block");
	}
	// set old state again
	i2c_Autoreading(state);

}

/*
 * loads config values from the external i2c eeprom
 */
void eeprom_LoadConfig(void) {
#ifdef DEBUG_UART
	stdComm_puts("load config...\n");
#endif
//	// save current state
//	FunctionalState state = internali2c.autoReading;
//	// disable auto reading and wait for the current cycle to finish
//	i2c_Autoreading(DISABLE);
//	while (internali2c.autoReadingRunning)
//		;
//	internali2c.error = RESET;
	// read eeprom
	eeprom_ReadBlock(0, (uint8_t*) &config, sizeof(config));
	// check for error
	if (internali2c.error == SET) {
		internali2c.error = RESET;
#ifdef DEBUG_UART
		stdComm_puts("I2C error eeprom\n");
#endif
		log_LogFileEntry("CRITICAL ERROR: I2C error while loading config");
		eeprom.configLoaded = RESET;
	} else {
#ifdef DEBUG_UART
		stdComm_puts("config loaded\n");
#endif
		log_LogFileEntry("eeprom configuration loaded");
		eeprom.configLoaded = SET;
	}
//	// set old state again
//	i2c_Autoreading(state);
}
/*
 * saves the config values to the eeprom
 */
void eeprom_SaveConfig(void) {
#ifdef DEBUG_UART
	stdComm_puts("save config...\n");
#endif
//// save current state
//	FunctionalState state = internali2c.autoReading;
//// disable auto reading and wait for the current cycle to finish
//	i2c_Autoreading(DISABLE);
//	while (internali2c.autoReadingRunning)
//		;
//	internali2c.error = RESET;
	uint16_t writtenBytes = 0;
	uint16_t configSize = sizeof(config);
	uint8_t pagecount = 0;
	while (configSize - writtenBytes > 0) {
		uint16_t blocksize = (configSize - writtenBytes);
		if (blocksize > EEPROM_PAGE_SIZE)
			blocksize = EEPROM_PAGE_SIZE;
		uint8_t *data = ((uint8_t*) &config);
		uint16_t i;
		for (i = 0; i < pagecount * EEPROM_PAGE_SIZE; i++)
			data++;
		eeprom_WritePage(pagecount * EEPROM_PAGE_SIZE, data, blocksize);
		writtenBytes += blocksize;
		pagecount++;
	}
// check for error
	if (internali2c.error == SET) {
		internali2c.error = RESET;
#ifdef DEBUG_UART
		stdComm_puts("I2C error eeprom\n");
#endif
		log_LogFileEntry("ERROR: I2C error while saving config");
	} else {
#ifdef DEBUG_UART
		stdComm_puts("config saved\n");
#endif
		log_LogFileEntry("configuration saved");
	}
//// set old state again
//	i2c_Autoreading(state);
}
