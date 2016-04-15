#include "log.h"

/*
 * checks for an SD card and initializes it. Furthermore it creates
 * the necessary directories (if not already existent) and logfiles.
 */
void log_Init(void) {
	//set pointers to logValues
	logfile.flightLogEntry[0] = &attitude.pitch;
	logfile.flightLogEntry[1] = &attitude.roll;
	logfile.flightLogEntry[2] = &attitude.yaw;
	logfile.flightLogEntry[3] = &accelerometer.X;
	logfile.flightLogEntry[4] = &accelerometer.Y;
	logfile.flightLogEntry[5] = &accelerometer.Z;
	logfile.flightLogEntry[6] = &accelerometer.magnitude;
	logfile.flightLogEntry[7] = &magnetometer.X;
	logfile.flightLogEntry[8] = &magnetometer.Y;
	logfile.flightLogEntry[9] = &magnetometer.Z;
	logfile.flightLogEntry[10] = &magnetometer.magnitude;
	logfile.flightLogEntry[11] = &gyro.X;
	logfile.flightLogEntry[12] = &gyro.Y;
	logfile.flightLogEntry[13] = &gyro.Z;
	logfile.flightLogEntry[14] = &gps.position.X;
	logfile.flightLogEntry[15] = &gps.position.Y;
	logfile.flightLogEntry[16] = &gps.position.Z;
	logfile.flightLogEntry[17] = &gps.velocity.X;
	logfile.flightLogEntry[18] = &gps.velocity.Y;
	logfile.flightLogEntry[19] = &pressure.height;
	logfile.flightLogEntry[20] = &externalSensor.angle;
	logfile.flightLogEntry[21] = &position.X;
	logfile.flightLogEntry[22] = &position.velocity.X;
	logfile.flightLogEntry[23] = &position.acceleration.X;
	logfile.flightLogEntry[24] = &position.Y;
	logfile.flightLogEntry[25] = &position.velocity.Y;
	logfile.flightLogEntry[26] = &position.acceleration.Y;
	logfile.flightLogEntry[27] = &position.Z;
	logfile.flightLogEntry[28] = &position.velocity.Z;
	logfile.flightLogEntry[29] = &position.acceleration.Z;
	logfile.flightLogEntry[30] = &distance.bottom;
	logfile.flightLogEntry[31] = &flightState.howerPower;
	// pointer to integer log values
	logfile.flightLogEntry2[0] = (int32_t*) &battery.voltage;
	logfile.flightLogEntry2IntType[0] = LOG_INT_UINT16;
	logfile.flightLogEntry2[1] = (int32_t*) &battery.usedCapacity;
	logfile.flightLogEntry2IntType[1] = LOG_INT_UINT16;
	logfile.flightLogEntry2[2] = (int32_t*) &battery.current;
	logfile.flightLogEntry2IntType[2] = LOG_INT_UINT16;
	logfile.flightLogEntry2[3] = (int32_t*) &gps.latitude;
	logfile.flightLogEntry2IntType[3] = LOG_INT_INT32;
	logfile.flightLogEntry2[4] = (int32_t*) &gps.longitude;
	logfile.flightLogEntry2IntType[4] = LOG_INT_INT32;
	logfile.flightLogEntry2[5] = (int32_t*) &gps.altitude;
	logfile.flightLogEntry2IntType[5] = LOG_INT_INT32;
	logfile.flightLogEntry2[6] = (int32_t*) &gps.heading;
	logfile.flightLogEntry2IntType[6] = LOG_INT_UINT16;
	logfile.flightLogEntry2[7] = (int32_t*) &gps.speed;
	logfile.flightLogEntry2IntType[7] = LOG_INT_UINT16;
	logfile.flightLogEntry2[8] = (int32_t*) &pressure.ADCValue;
	logfile.flightLogEntry2IntType[8] = LOG_INT_INT32;

	// check PD10 (SD card detector switch)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &gpio);

	// initialize variables
	logfile.logFileRunning = 0;
	logfile.entryBufferRead = 0;
	logfile.entryBufferWrite = 0;
	logfile.cardAvailable = 1;
	logfile.cardMissingCounter = 0;

	if (log_CheckIfCardAvailable()) {
		// card is available -> try to mount it
		if (f_mount(&logfile.fatfs, "", 1) != FR_OK) {
			stdComm_puts("failed to mount SD card\n");
			return;
		}

		// try to create folder "SYSLOGS"
		FRESULT res = f_mkdir("SYSLOGS");
		if (res != FR_OK && res != FR_EXIST) {
			stdComm_puts("failed to create folder 'SYSLOGS'\n");
			return;
		}
		// change into directory
		if (f_chdir("/SYSLOGS") != FR_OK) {
			stdComm_puts("failed to enter folder 'SYSLOGS'\n");
			return;
		}
		/*
		 * Name format for logfiles is LOGXXX.TXT, where XXX is a 3-digit number.
		 * Already existent files are skipped.
		 */
		TCHAR logfilename[] = "LOG000.TXT";
		uint8_t fileAlreadyExists = 1;
		logfilename[3] = '0' - 1;
		do {
			logfilename[3]++;
			logfilename[4] = '0' - 1;
			do {
				logfilename[4]++;
				logfilename[5] = '0' - 1;
				do {
					logfilename[5]++;
					if (f_open(&logfile.logFile, logfilename,
					FA_WRITE | FA_CREATE_NEW) == FR_OK) {
						fileAlreadyExists = 0;
					}
				} while (logfilename[5] < '9' && fileAlreadyExists);
			} while (logfilename[4] < '9' && fileAlreadyExists);
		} while (logfilename[3] < '9' && fileAlreadyExists);
		if (fileAlreadyExists) {
			// wasn't able to create file
			stdComm_puts("failed to create logfile\n");
			return;
		}
		// now the logfile is created
		logfile.logFileRunning = 1;
		led_Cmd(4, LED_ON);
		stdComm_puts("started logfile:");
		stdComm_puts(logfilename);
		usart_putcStdComm('\n');

		// change into directory
		if (f_chdir("/") != FR_OK) {
			stdComm_puts("failed to return to root\n");
			return;
		}

		// check for flag file
		FIL flagFile;
		if (f_open(&flagFile, "FLAGS", FA_READ | FA_OPEN_EXISTING) == FR_OK) {
			log_LogFileEntry("evaluating flagfile...");
			char lineBuffer[50];
			char data;
			UINT charsRead = 0;
			uint8_t charCount = 0;
			while (f_read(&flagFile, &data, 1, &charsRead) == FR_OK) {
				if(charsRead==0){
					// reached end of file
					log_LogFileEntry("...done");
					break;
				}
				if (data == '\n') {
					// newline detected
					// compare lineBuffer to known flags
					if(StringEquals(lineBuffer, "BARO_TEMP_PROFILE")){
						logfile.flags.baroTempProfileActive = 1;
						log_LogFileEntry("flag set: BARO_TEMP_PROFILE");
					}
					charCount = 0;
				} else {
					lineBuffer[charCount] = data;
					charCount++;
					if(charCount>=50){
						// line is too long
						// -> start over too avoid buffer overflow
						charCount = 0;
					}
				}
			}
			f_close(&flagFile);
		} else {
			// no flag file found -> disable all flags
			logfile.flags.baroTempProfileActive = 0;
			log_LogFileEntry("no flagfile available");
		}

		// try to create folder "FLIGHTS"
		res = f_mkdir("/FLIGHTS");
		if (res != FR_OK && res != FR_EXIST) {
			stdComm_puts("failed to create folder 'FLIGHTS'\n");
			return;
		}
		// change into directory
		if (f_chdir("/FLIGHTS") != FR_OK) {
			stdComm_puts("failed to enter folder 'FLIGHTS'\n");
			return;
		}
	} else {
		stdComm_puts("no SD card available\n");
		logfile.flags.baroTempProfileActive = 0;
	}
}
uint8_t log_CheckIfCardAvailable(void) {
	if (GPIOD->IDR & GPIO_Pin_10) {
		// no card detected, but this might be just a glitch in the switch
		if (logfile.cardMissingCounter) {
			logfile.cardMissingCounter--;
		} else {
			// at least 10 consecutive "high" reads -> card is missing
			if (logfile.cardAvailable)
				stdComm_puts("SD card has been removed\n");
			logfile.cardAvailable = 0;
			logfile.flightLogRunning = 0;
			logfile.logFileRunning = 0;
			logfile.flags.baroTempProfileActive = 0;
			led_Cmd(4, LED_OFF);
		}
	} else {
		if (!logfile.cardAvailable) {
			stdComm_puts("SD card has been inserted\n");
			logfile.cardAvailable = 1;
			log_Init();
		}
		logfile.cardMissingCounter = 10;
	}
	return logfile.cardAvailable;
}
/*
 * prepares a line to be entered into the logfile.
 */
void log_LogFileEntry(char* entry) {
	if (logfile.logFileRunning) {
		// check for enough buffer space
		int8_t freeBufferSpace = logfile.entryBufferRead
				- logfile.entryBufferWrite;
		if (freeBufferSpace <= 0)
			freeBufferSpace += LOG_ENTRY_BUFFER_SIZE;
		if (freeBufferSpace >= 2) {
			// get current time
			logfile.entryTimestamp[logfile.entryBufferWrite] = time_GetMillis();
			// copy entry into buffer
			uint8_t i = 0;
			while (i < LOG_ENTRY_BUFFER_LENGTH - 2 && *entry != 0) {
				logfile.entryBuffer[logfile.entryBufferWrite][i] = *entry++;
				i++;
			}
			logfile.entryBuffer[logfile.entryBufferWrite][i] = '\r';
			logfile.entryBuffer[logfile.entryBufferWrite][i + 1] = '\n';
			i += 2;
			// set entry length
			logfile.entryLength[logfile.entryBufferWrite] = i;
			// increase buffer write position
			logfile.entryBufferWrite++;
			logfile.entryBufferWrite %= LOG_ENTRY_BUFFER_SIZE;
		}
	}
}
/*
 * writes the buffered entries into the logfile
 */
void log_FlushBufferedEntries(void) {
	if (logfile.logFileRunning) {
		while (logfile.entryBufferRead != logfile.entryBufferWrite) {
			// write timestamp
			uint8_t minutes, seconds;
			uint32_t millis;
			millis = logfile.entryTimestamp[logfile.entryBufferRead];
			minutes = millis / 60000;
			seconds = (millis / 1000) % 60;
			millis %= 1000;
			char timeString[10];
			/*
			 * timeString format:
			 * 'mm:ss.mmm '
			 */
			timeString[0] = minutes / 10 + '0';
			timeString[1] = minutes % 10 + '0';
			timeString[2] = ':';
			timeString[3] = seconds / 10 + '0';
			timeString[4] = seconds % 10 + '0';
			timeString[5] = '.';
			timeString[6] = millis / 100 + '0';
			timeString[7] = (millis / 10) % 10 + '0';
			timeString[8] = millis % 10 + '0';
			timeString[9] = ' ';
			UINT writtenBytes;
			f_write(&logfile.logFile, timeString, 10, &writtenBytes);
			f_write(&logfile.logFile,
					logfile.entryBuffer[logfile.entryBufferRead],
					logfile.entryLength[logfile.entryBufferRead],
					&writtenBytes);
			// increase read position
			logfile.entryBufferRead++;
			logfile.entryBufferRead %= LOG_ENTRY_BUFFER_SIZE;
		}
	}
}

/*
 * flushes all data into logfiles
 */
void log_SyncFileSystem(void) {
	if (logfile.logFileRunning) {
		f_sync(&logfile.logFile);
	}
//	if(logfile.flightLogRunning){
//		f_sync(&logfile.flightLog);
//	}
}

/*
 * requests the start of a new flightLog
 */
void log_FlightLogStartRequest(void) {
	logfile.flightLogStartRequest = 1;
	logfile.flightLogStartTimestamp = time_GetMillis();
}

/*
 * requests the end of the current flightLog
 */
void log_FlightLogStopRequest(void) {
	if (logfile.flightLogRunning)
		logfile.flightLogStopRequest = 1;
}

/*
 * starts a new flightLog.
 * !! DO NOT USE THIS FUNCTION IN AN INTERRUPT !!
 * use log_FlightLogStartRequest instead
 */
void log_StartFlightLog(void) {
	logfile.flightLogStartRequest = 0;
	// close the potentially running flightLog
	if (logfile.flightLogRunning)
		log_StopFlightLog();
	/*
	 * Name format for flightLogs is FLLOGXXX.CSV, where XXX is a 3-digit number.
	 * Already existent files are skipped.
	 */
	TCHAR logfilename[] = "FLLOG000.CSV";
	uint8_t fileAlreadyExists = 1;
	logfilename[5] = '0' - 1;
	do {
		logfilename[5]++;
		logfilename[6] = '0' - 1;
		do {
			logfilename[6]++;
			logfilename[7] = '0' - 1;
			do {
				logfilename[7]++;
				if (f_open(&logfile.flightLog, logfilename,
				FA_WRITE | FA_CREATE_NEW) == FR_OK) {
					fileAlreadyExists = 0;
				}
			} while (logfilename[7] < '9' && fileAlreadyExists);
		} while (logfilename[6] < '9' && fileAlreadyExists);
	} while (logfilename[5] < '9' && fileAlreadyExists);
	if (fileAlreadyExists) {
		// wasn't able to create file
		log_LogFileEntry("ERROR: failed to create flightLog");
		return;
	}
	logfile.flightLogRunning = 1;
	char logentry[] = "created flightLog:        ";
	uint8_t i;
	for (i = 0; i < 8; i++) {
		logentry[i + 18] = logfilename[i];
	}
	log_LogFileEntry(logentry);
	/*
	 * write title line of flightLog
	 */
	// float values
	char logValueName[LOG_MAXIMUM_LOGVALUES_FLOAT][15] = { "pitch", "roll",
			"yaw", "accX", "accY", "accZ", "accMagnitude", "magX", "magY",
			"magZ", "magMagnitude", "gyroX", "gyroY", "gyroZ", "GPS X", "GPS Y",
			"GPS Z", "GPSvelX", "GPSvelY", "baroHeight", "extTilt", "PositionX",
			"VelocityX", "accelerationX", "PositionY", "VelocityY",
			"accelerationY", "PositionZ", "VelocityZ", "accelerationZ",
			"distanceBottom", "howerpower" };
	char logline[2000];
	char *p = logline;
	uint32_t mask = config.flightLogMask;
	for (i = 0; i < LOG_MAXIMUM_LOGVALUES_FLOAT; i++) {
		if (mask & 0x01) {
			p = StringCopy(p, logValueName[i]);
			*p++ = ';';
		}
		mask >>= 1;
	}
	// integer values
	char logValueName2[LOG_MAXIMUM_LOGVALUES_INT][15] = { "batVoltage",
			"usedCapacity", "current", "GPSLatitude", "GPSLongitude",
			"GPSAltitude", "GPSHeading", "GPSSpeed", "rawPressureADC" };
	mask = config.flightLogMask2;
	for (i = 0; i < LOG_MAXIMUM_LOGVALUES_INT; i++) {
		if (mask & 0x01) {
			p = StringCopy(p, logValueName2[i]);
			*p++ = ';';
		}
		mask >>= 1;
	}
	p--;
	*p++ = '\r';
	*p++ = '\n';
	f_write(&logfile.flightLog, logline, p - logline, 0);
}

/*
 * stops the current flightLog
 *  * !! DO NOT USE THIS FUNCTION IN AN INTERRUPT !!
 * use log_FlightLogStopRequest instead
 */
void log_StopFlightLog(void) {
	logfile.flightLogStopRequest = 0;
	if (logfile.flightLogRunning) {
		// close file
		f_close(&logfile.flightLog);
		logfile.flightLogRunning = 0;
		log_LogFileEntry("stopped flightLog");
	}
}

/*
 * writes a new data set into the flightLog
 */
void log_FlightLogAddEntry(void) {
	if (logfile.flightLogRunning) {
		char logline[2000];
		char *p = logline;
		uint32_t mask = config.flightLogMask;
		uint8_t i;
		for (i = 0; i < LOG_MAXIMUM_LOGVALUES_FLOAT; i++) {
			if (mask & 0x01) {
				p = ftoa(*logfile.flightLogEntry[i], p);
				*p++ = ';';
			}
			mask >>= 1;
		}
		mask = config.flightLogMask2;
		for (i = 0; i < LOG_MAXIMUM_LOGVALUES_INT; i++) {
			if (mask & 0x01) {
				int32_t value = 0;
				switch (logfile.flightLogEntry2IntType[i]) {
				case LOG_INT_UINT8:
					value = *(uint8_t*) logfile.flightLogEntry2[i];
					break;
				case LOG_INT_INT8:
					value = *(int8_t*) logfile.flightLogEntry2[i];
					break;
				case LOG_INT_UINT16:
					value = *(uint16_t*) logfile.flightLogEntry2[i];
					break;
				case LOG_INT_INT16:
					value = *(int16_t*) logfile.flightLogEntry2[i];
					break;
				case LOG_INT_INT32:
					value = *logfile.flightLogEntry2[i];
					break;
				}
				p = itoASCII(value, p);
				*p++ = ';';
			}
			mask >>= 1;
		}
		p--;
		*p++ = '\r';
		*p++ = '\n';
		f_write(&logfile.flightLog, logline, p - logline, 0);
	}
}

/*******************************************************************
 * kalmanlog functions
 ******************************************************************/
/*
 * requests the start of a new kalmanLog
 */
void log_KalmanLogStartRequest(void) {
	logfile.kalmanLogStartRequest = 1;
}

/*
 * requests the end of the current kalmanLog
 */
void log_KalmanLogStopRequest(void) {
	if (logfile.kalmanLogRunning)
		logfile.kalmanLogStopRequest = 1;
}

/*
 * starts a new kalmanLog.
 * !! DO NOT USE THIS FUNCTION IN AN INTERRUPT !!
 * use log_KalmanLogStartRequest instead
 */
void log_StartKalmanLog(void) {
	logfile.kalmanLogStartRequest = 0;
	// close the potentially running flightLog
	if (logfile.kalmanLogRunning)
		log_StopKalmanLog();
	/*
	 * Name format for flightLogs is KLLOGXXX.BIN, where XXX is a 3-digit number.
	 * Already existent files are skipped.
	 */
	TCHAR logfilename[] = "KLLOG000.BIN";
	uint8_t fileAlreadyExists = 1;
	logfilename[5] = '0' - 1;
	do {
		logfilename[5]++;
		logfilename[6] = '0' - 1;
		do {
			logfilename[6]++;
			logfilename[7] = '0' - 1;
			do {
				logfilename[7]++;
				if (f_open(&logfile.kalmanLog, logfilename,
				FA_WRITE | FA_CREATE_NEW) == FR_OK) {
					fileAlreadyExists = 0;
				}
			} while (logfilename[7] < '9' && fileAlreadyExists);
		} while (logfilename[6] < '9' && fileAlreadyExists);
	} while (logfilename[5] < '9' && fileAlreadyExists);
	if (fileAlreadyExists) {
		// wasn't able to create file
		log_LogFileEntry("ERROR: failed to create kalmanLog");
		return;
	}
	logfile.kalmanLogRunning = 1;
	logfile.kalmanBufferRead = 0;
	logfile.kalmanBufferWrite = 0;
	char logentry[] = "created KalmanLog:        ";
	uint8_t i;
	for (i = 0; i < 8; i++) {
		logentry[i + 18] = logfilename[i];
	}
	log_LogFileEntry(logentry);
	logfile.kalmanLogEntryNumber = 0;
	/*
	 * write header of kalmanLogFile
	 */
	uint16_t identifier = 0x4b4c;
	struct {
		float SampleTime;
		float AccNoise;
		float MagNoise;
		float GyroNoise;
		float StateAccNoise;
		float StateMagNoise;
		float StateGyroNoise;
		float StateGyroBiasNoise;
	} kalmanLogHeader = { 1.0f / I2C_CYCLE_FREQUENCY,
			config.attitudeKalman.CovMeasAcc, config.attitudeKalman.CovMeasMag,
			config.attitudeKalman.CovMeasGyro,
			config.attitudeKalman.CovStateAcc,
			config.attitudeKalman.CovStateMag,
			config.attitudeKalman.CovStateGyro,
			config.attitudeKalman.CovStateGyroBias };
	f_write(&logfile.kalmanLog, &identifier, sizeof(identifier), 0);
	f_write(&logfile.kalmanLog, &kalmanLogHeader, sizeof(kalmanLogHeader), 0);
}

/*
 * stops the current kalmanLog
 * !! DO NOT USE THIS FUNCTION IN AN INTERRUPT !!
 * use log_kalmanLogStopRequest instead
 */
void log_StopKalmanLog(void) {
	logfile.kalmanLogStopRequest = 0;
	if (logfile.kalmanLogRunning) {
		// close file
		f_close(&logfile.kalmanLog);
		logfile.kalmanLogRunning = 0;
		log_LogFileEntry("stopped kalmanLog");
	}
}

/*
 * adds a new set of data to the kalman log buffer
 */
void log_KalmanLogAddEntry(void) {
	if (logfile.kalmanLogRunning) {
		// check for enough buffer space
		int8_t freeBufferSpace = logfile.kalmanBufferRead
				- logfile.kalmanBufferWrite;
		if (freeBufferSpace <= 0)
			freeBufferSpace += LOG_KALMAN_BUFFER_LENGTH;
		if (freeBufferSpace >= 2) {
			logfile.kalmanEntries[logfile.kalmanBufferWrite].accX =
					accelerometer.X;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].accY =
					accelerometer.Y;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].accZ =
					accelerometer.Z;

			logfile.kalmanEntries[logfile.kalmanBufferWrite].magX =
					magnetometer.X;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].magY =
					magnetometer.Y;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].magZ =
					magnetometer.Z;

			logfile.kalmanEntries[logfile.kalmanBufferWrite].gyroX = gyro.X;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].gyroY = gyro.Y;
			logfile.kalmanEntries[logfile.kalmanBufferWrite].gyroZ = gyro.Z;

			logfile.kalmanEntries[logfile.kalmanBufferWrite].trueAngle =
					externalSensor.angle;

			logfile.kalmanEntries[logfile.kalmanBufferWrite].number =
					logfile.kalmanLogEntryNumber;
			logfile.kalmanBufferWrite++;
			logfile.kalmanBufferWrite %= LOG_KALMAN_BUFFER_LENGTH;
		}
		logfile.kalmanLogEntryNumber++;
	}
}

/*
 * writes buffered entries into kalmanlog
 */
void log_KalmanLogFlushEntries(void) {
	if (logfile.kalmanLogRunning) {
		uint8_t writtenSets = 0;
		while (logfile.kalmanBufferRead != logfile.kalmanBufferWrite
				&& writtenSets < LOG_KALMAN_BUFFER_LENGTH) {
			// write data
			f_write(&logfile.kalmanLog,
					&logfile.kalmanEntries[logfile.kalmanBufferRead],
					sizeof(struct KalmanLogEntry), 0);
			// increase read position
			logfile.kalmanBufferRead++;
			logfile.kalmanBufferRead %= LOG_KALMAN_BUFFER_LENGTH;
			writtenSets++;
		}

	}
}
