/*
 * crc8.c
 *
 *  Created on: 13.01.2018
 *      Author: miczy
 */

/* CRC* Dallas/Maxim for Si7021 temperature and humidity sensor
 *
 */
#include "crc8.h"

uint8_t Crc8_Calculate(uint8_t *data, uint16_t dataSize, uint8_t initValue)
{
	if (!data|| dataSize == 0)
	        return 0;

	unsigned int index = 0;
	uint8_t crc = initValue;

	while(index < dataSize) {
		crc = crc8_Table[crc ^ (*( data + index ))] ;
		index++;
	}

	return crc;
}
