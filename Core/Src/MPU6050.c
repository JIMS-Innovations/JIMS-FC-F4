/**
 * @file MPU6050.c
 *
 * @author Jesutofunmi Kupoluyi (innovationsjims@gmail.com)
 *
 * @brief This is the function definition file for the MPU6050
 * @version 0.3
 * @date 2023-04-05
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Including required header file*/
#include "MPU6050.h"

/*
 * LOW LEVEL FUNCTIONS
 */

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT);

}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg,
		uint8_t *data, uint8_t length) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);

}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg,
		uint8_t *data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT);

}

/*
 * INITIALIZATION
 */

uint8_t MPU6050_Initialise(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set struct parameters */
	dev->i2cHandle = i2cHandle;
	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->gyro_deg[0] = 0.0f;
	dev->gyro_deg[1] = 0.0f;
	dev->gyro_deg[2] = 0.0f;

	dev->temp_C = 0.0f;

	/* Number of errors */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData = 0;

	/*
	 * Check device ID
	 */
	status = MPU6050_ReadRegister(dev, WHO_AM_I, &regData);
	errNum += (status != HAL_OK);

	if (regData != 0x68) {

//		return 255;

	}

	/*
	 * Putting the sensor on
	 */

	regData = 0X00;
	status = MPU6050_WriteRegister(dev, PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);

	/* Using the on chip digital low pass filter */
	regData = 0X02;
	status = MPU6050_WriteRegister(dev, CONFIG, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Configuring the gyro
	 */
	regData = 0X08;
	status = MPU6050_WriteRegister(dev, GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Configuring the accelerometer
	 */
	regData = 0X10;
	status = MPU6050_WriteRegister(dev, ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);

	/* Return the number of errors */
	return errNum;

}

/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050 *dev) {

	/* Reading raw temperature values */
	uint8_t regData[2];

	HAL_StatusTypeDef status;

	status = MPU6050_ReadRegisters(dev, TEMP_OUT_H, regData, 2);

	/*
	 * Combine values to give raw temperature reading
	 */
	int16_t tempRaw = (regData[0] << 8 | regData[1]);

	/*
	 * Convert to •C
	 */
	dev->temp_C = ((float) tempRaw / 340) + 36.5;

	return status;

}

HAL_StatusTypeDef MPU6050_ReadAccelerations(MPU6050 *dev) {

	/* Reading raw accelerometer values */
	uint8_t regData[6];

	HAL_StatusTypeDef status;

	status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, regData, 6);

	/*
	 * Combine values to give raw temperature reading
	 */
	int16_t accRaw[3];

	accRaw[0] = (regData[0] << 8 | regData[1]);
	accRaw[1] = (regData[2] << 8 | regData[3]);
	accRaw[2] = (regData[4] << 8 | regData[5]);

	/*
	 * Convert to mps^2 (given range setting of +/- 8g)
	 * Remapping of Axes to support EKF standard axes configuration
	 */
//	dev->acc_mps2[0] = -0.00239501953125 * accRaw[1];
//	dev->acc_mps2[1] = -0.00239501953125 * accRaw[0];
//	dev->acc_mps2[2] = -0.00239501953125 * accRaw[2];

	/* Worked for complementary filter */
	dev->acc_mps2[0] = 0.00239501953125 * accRaw[0];
	dev->acc_mps2[1] = 0.00239501953125 * accRaw[1];
	dev->acc_mps2[2] = 0.00239501953125 * accRaw[2];

	return status;
}

HAL_StatusTypeDef MPU6050_ReadOrientation(MPU6050 *dev) {

	/* Reading raw accelerometer values */
	uint8_t regData[6];

	HAL_StatusTypeDef status;

	status = MPU6050_ReadRegisters(dev, GYRO_XOUT_H, regData, 6);

	/*
	 * Combine values to give raw temperature reading
	 */
	int16_t gyroRaw[3];

	gyroRaw[0] = (regData[0] << 8 | regData[1]);
	gyroRaw[1] = (regData[2] << 8 | regData[3]);
	gyroRaw[2] = (regData[4] << 8 | regData[5]);

	/*
	 * Convert to (deg/s ==> rad/s ==> rad [in a 1kHz loop]) (given range setting of +/- 500•/s)
	 */
//	dev->gyro_deg[0] += 0.0655 * 0.0174533 * gyroRaw[0];
//	dev->gyro_deg[1] += 0.0655 * 0.0174533 * gyroRaw[1];
//	dev->gyro_deg[2] += 0.0655 * 0.0174533 * gyroRaw[2];

	/* Worked for complementary filter */
	dev->gyro_deg[0] = 0.00026646259542f * gyroRaw[0];
	dev->gyro_deg[1] = 0.00026646259542f * gyroRaw[1];
	dev->gyro_deg[2] = 0.00026646259542f * gyroRaw[2];

	/* deg/s ==> rad/s  ==> remapping the axes */
//	dev->gyro_deg[0] = -0.00026646259542f * gyroRaw[1];
//	dev->gyro_deg[1] = -0.00026646259542f * gyroRaw[0];
//	dev->gyro_deg[2] = -0.00026646259542f * gyroRaw[2];

	return status;
}

