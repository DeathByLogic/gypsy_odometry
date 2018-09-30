#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "beagleIO.h"
#include "encoders.h"

using namespace std;

//
// Global variables
//

void read_encoder(void *memAdr, size_t size, const char i2cAddress) {
	char start_adr[1] = { 0x00 };
	char tempArray[size];

	// Get data from left sensors
	for (int i = 0; i < READ_FAIL_COUNT; i++) {
		//ENCODER_I2C.writeTo(start_adr, i2cAddress, sizeof(start_adr));
		//ENCODER_I2C.requestFrom(tempArray, i2cAddress, size, true);

		if (verify_crc(tempArray, sizeof(tempArray), 0x0000)) {
			// Copy data into sensor struct
			memcpy(memAdr, &tempArray, size);

			break;
		} else {
			// Clear memory if invalid data
			memset(memAdr, 0x00, size);
		}
	}
}

bool verify_crc(char *str, size_t size, const unsigned short crc) {
	unsigned short calc_crc = 0x0000;

	// Calculate the CRC of the data
	for (unsigned int i = 0; i < size; i++) {
		calc_crc = crc_xmodem_update(calc_crc, str[i]);

		//printf("Byte %i: %X\t CRC: %X\n", i, str[i], calc_crc);
	}

	// Check if calculated CRC matches supplied
	if (calc_crc == crc) {
		return true;
	} else {
		return false;
	}
}

unsigned short crc_xmodem_update (unsigned short crc, char data) {
	crc = crc ^ ((unsigned short)data << 8);

	for (int i=0; i< 8; i++) {
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}

	return crc;
}
