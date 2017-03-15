// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef PORT_H
#define PORT_H
#include <stdint.h>

class Port
{
public:
	// write to the port, return number of bytes written or -1 if error.
	virtual int write(const uint8_t* ptr, int count) = 0;

	// read a given number of bytes from the port (blocking until the requested bytes are available).
	// return the number of bytes read or -1 if error.
	virtual int read(uint8_t* buffer, int bytesToRead) = 0;

	// close the port.
	virtual void close() = 0;

	virtual bool isClosed() = 0;

    virtual int getRssi(const char* ifaceName) = 0;

};
#endif // !PORT_H
