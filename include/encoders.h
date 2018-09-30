#ifndef _ENCODERS_H
#define _ENCODERS_H

// Constant Definitions
#define ENCODER_I2C			P9_20

#define READ_FAIL_COUNT		3

//Function Prototypes
void read_encoder(void *memAdr, size_t size, const char i2cAddress);
bool verify_crc(char *str, size_t size, const unsigned short crc);
unsigned short crc_xmodem_update (unsigned short crc, char data);

#endif
