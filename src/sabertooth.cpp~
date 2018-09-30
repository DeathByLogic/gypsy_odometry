/*
Arduino Library for SyRen/Sabertooth Packet Serial
Copyright (c) 2012 Dimension Engineering LLC
http://www.dimensionengineering.com/arduino

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
USE OR PERFORMANCE OF THIS SOFTWARE.

Modified for use with the beagleIO library and the Beagle Bone board
by daniel@deathbylogic.com.
*/

#include <stdlib.h>
#include <unistd.h>
#include <algorithm>

#include "Sabertooth.h"

using namespace std;

Sabertooth::Sabertooth(uint8_t address, beagleSerial *port)
  : _address(address), _port(port)
{

}

void Sabertooth::autobaud(bool dontWait) const
{
	autobaud(port(), dontWait);
}

void Sabertooth::autobaud(beagleSerial *port, bool dontWait)
{
	const char ab = 0xAA;

	if (!dontWait) { usleep(1500000); }

	port->writePort(&ab, 1);

	if (!dontWait) { usleep(500000); }
}

void Sabertooth::command(uint8_t command, uint8_t value) const
{
	SabertoothPacket(command, value).send(*this);
}

void Sabertooth::motor(int power) const
{
	SabertoothPacket::motor(power).send(*this);
}

void Sabertooth::motor(uint8_t motor, int power) const
{
	SabertoothPacket::motor(motor, power).send(*this);
}

void Sabertooth::drive(int power) const
{
	SabertoothPacket::drive(power).send(*this);
}

void Sabertooth::turn(int power) const
{
	SabertoothPacket::turn(power).send(*this);
}

void Sabertooth::stop() const
{
	motor(1, 0);
	motor(2, 0);
}

void Sabertooth::setMinVoltage(uint8_t value) const
{
	SabertoothPacket::setMinVoltage(value).send(*this);
}

void Sabertooth::setMaxVoltage(uint8_t value) const
{
	SabertoothPacket::setMaxVoltage(value).send(*this);
}

void Sabertooth::setBaudRate(long baudRate) const
{
#if defined(ARDUINO) && ARDUINO >= 100
	//port().flush();
#endif

	SabertoothPacket::setBaudRate(baudRate).send(*this);

#if defined(ARDUINO) && ARDUINO >= 100
	//port().flush();
#endif
  
	// (1) flush() does not seem to wait until transmission is complete.
	//     As a result, a Serial.end() directly after this appears to
	//     not always transmit completely. So, we manually add a delay.
	// (2) Sabertooth takes about 200 ms after setting the baud rate to
	//     respond to commands again (it restarts).
	// So, this 500 ms delay should deal with this.
	usleep(500000);
}

void Sabertooth::setDeadband(uint8_t value) const
{
	SabertoothPacket::setDeadband(value).send(*this);
}

void Sabertooth::setRamping(uint8_t value) const
{
	SabertoothPacket::setRamping(value).send(*this);
}

void Sabertooth::setTimeout(int milliseconds) const
{
	SabertoothPacket::setTimeout(milliseconds).send(*this);
}

SabertoothPacket::SabertoothPacket(uint8_t command, uint8_t value)
	: _command(command), _value(value)
{
  
}

uint8_t SabertoothPacket::checksum(uint8_t address) const
{
	return (address + command() + value()) & 0b01111111;
}

void SabertoothPacket::getBytes(uint8_t address, uint8_t bytes[4]) const
{
	bytes[0] = address;
	bytes[1] = command();
	bytes[2] = value();
	bytes[3] = checksum(address);
}

void SabertoothPacket::send(uint8_t address, beagleSerial *port) const
{
	uint8_t bytes[4];

	getBytes(address, bytes);

	port->writePort(bytes, 4);
}

void SabertoothPacket::send(const Sabertooth& sabertooth) const
{
	send(sabertooth.address(), sabertooth.port());
}

SabertoothPacket SabertoothPacket::motor(int power)
{
	return motor(1, power);
}

SabertoothPacket SabertoothPacket::motor(uint8_t motor, int power)
{
	return throttleType((motor == 2 ? 4 : 0) + (power < 0 ? 1 : 0), power);
}

SabertoothPacket SabertoothPacket::drive(int power)
{
	return throttleType(power < 0 ? 9 : 8, power);
}

SabertoothPacket SabertoothPacket::turn(int power)
{
	return throttleType(power < 0 ? 11 : 10, power);
}

SabertoothPacket SabertoothPacket::setMinVoltage(uint8_t value)
{
	return SabertoothPacket(2, (uint8_t)min((unsigned int)value, (unsigned int)120));
}

SabertoothPacket SabertoothPacket::setMaxVoltage(uint8_t value)
{
	return SabertoothPacket(3, (uint8_t)min((unsigned int)value, (unsigned int)127));
}

SabertoothPacket SabertoothPacket::setBaudRate(long baudRate)
{
	uint8_t value;

	switch (baudRate)
	{
		case 2400:           value = 1; break;
		case 9600: default:  value = 2; break;
		case 19200:          value = 3; break;
		case 38400:          value = 4; break;
		case 115200:         value = 5; break;
	}

	return SabertoothPacket(15, value);
}

SabertoothPacket SabertoothPacket::setDeadband(uint8_t value)
{
	return SabertoothPacket(17, (uint8_t)min((unsigned int)value, (unsigned int)127));
}

SabertoothPacket SabertoothPacket::setRamping(uint8_t value)
{
	return SabertoothPacket(16, (uint8_t)CONSTRAIN((unsigned int)value, (unsigned int)0, (unsigned int)80));
}

SabertoothPacket SabertoothPacket::setTimeout(int milliseconds)
{
	return SabertoothPacket(14, (uint8_t)((CONSTRAIN(milliseconds, 0, 12700) + 99) / 100));
}

SabertoothPacket SabertoothPacket::throttleType(uint8_t command, int power)
{
	power = CONSTRAIN(power, -127, 127);

	return SabertoothPacket(command, abs(power));
}
