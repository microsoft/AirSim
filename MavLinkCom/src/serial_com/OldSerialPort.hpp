#pragma once

#include <string>
#include "Port.h"

#ifdef _WIN32
#include "Windows.h"
#else
#include <termios.h>
#include <unistd.h>
#endif

enum Parity {
    Parity_None = (0x0100),
    Parity_Odd = (0x0200),
    Parity_Even = (0x0400),
    Parity_Mark = (0x0800),
    Parity_Space = (0x1000)
};

enum StopBits
{
    StopBits_None = 0,
    StopBits_10 = (0x0001),
    StopBits_15 = (0x0002),
    StopBits_20 = (0x0004)
};

enum Handshake
{
    Handshake_None,
    Handshake_XonXoff,
    Handshake_RequestToSend,
    Handshake_RequestToSendXonXoff
};

class SerialPort : public Port
{
public:
    SerialPort();
    ~SerialPort();

    // open the serial port
	int connect(const char* portName, int baudRate);

    // write to the serial port
    int write(const uint8_t* ptr, int count);

    // read a given number of bytes from the port.
	int read(uint8_t* buffer, int bytesToRead);

    // close the port.
    void close();

	virtual bool isClosed() {
		return closed_;
	}
private:
    int setAttributes(int baud_rate, Parity parity, int data_bits, StopBits bits, Handshake hs, int readTimeout, int writeTimeout);

	bool closed_;
#ifdef _WIN32
    HANDLE handle;
    OVERLAPPED writeOverlapped;
    OVERLAPPED readOverlapped;
#else
    int fd;
    int getSpeed();
    void setSpeed(int baudRate);
#endif
};
