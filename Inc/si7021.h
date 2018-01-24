/*
 * si7021.h
 *
 *  Created on: 13.01.2018
 *      Author: miczy
 */

#ifndef SI7021_H_
#define SI7021_H_

#include "stm32f3xx_hal.h"

#define SLAVE_ADDRESS (0x40 << 1)

typedef enum
{
	MEASURE_RH_HOLD			= 0xE5,
	MEASURE_RH_NOHOLD		= 0xF5,
	MEASURE_TEMP_HOLD		= 0xE3,
	MEASURE_TEMP_NOHOLD	 	= 0xF3,
	TEMP_FROM_PREV_RH		= 0xE0,
	SENSOR_RESET			= 0xFE,
	WRITE_USER_REG			= 0xE6,
	READ_USER_REG			= 0xE7,
	WRITE_HEATER_CTRL_REG	= 0x51,
	READ_HEATER_CTRL_REG	= 0x11,
} SI7021_Command;

void SI7021_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef  SI7021_Read_Temperature_Hold(uint8_t *Data, uint8_t crc);
HAL_StatusTypeDef  SI7021_Read_Humidity_Hold(uint8_t *Data, uint8_t crc);
uint8_t	convertToRH(uint8_t *humidity);
float convertToCelsius(uint8_t *temperature);


#endif /* SI7021_H_ */
