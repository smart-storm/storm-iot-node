/*
 * si7021.c
 *
 *  Created on: 13.01.2018
 *      Author: miczy
 */

#include "si7021.h"

static I2C_HandleTypeDef *i2c_handle;

void SI7021_Init(I2C_HandleTypeDef *hi2c)
{
	i2c_handle = hi2c;
}

HAL_StatusTypeDef  SI7021_Read_Temperature_Hold(uint8_t *Data, uint8_t crc)
{
	if(crc) {
		return HAL_I2C_Mem_Read(i2c_handle, SLAVE_ADDRESS, MEASURE_TEMP_HOLD, 1, Data, 3, 500);
	} else {
		return HAL_I2C_Mem_Read(i2c_handle, SLAVE_ADDRESS, MEASURE_TEMP_HOLD, 1, Data, 2, 500);
	}
}


HAL_StatusTypeDef  SI7021_Read_Humidity_Hold(uint8_t *Data, uint8_t crc)
{
	if(crc) {
		return HAL_I2C_Mem_Read(i2c_handle, SLAVE_ADDRESS, MEASURE_RH_HOLD, 1, Data, 3, 500);
	} else {
		return HAL_I2C_Mem_Read(i2c_handle, SLAVE_ADDRESS, MEASURE_RH_HOLD, 1, Data, 2, 500);
	}
}

uint8_t	convertToRH(uint8_t *humidity)
{
	static uint8_t RH_value = 0;
	RH_value = (125 * (((*humidity) << 8) | (*(humidity + 1))) / 65536) - 6;
	return RH_value;
}

float convertToCelsius(uint8_t *temperature)
{
	static float temp = 0.0;
	temp = (172.72 * (((*temperature) << 8) | (*(temperature + 1))) / 65536.0) - 46.85;
	return temp;
}


