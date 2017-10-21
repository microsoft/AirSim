// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "SerialPort.hpp"

#ifdef _WIN32
#include <comdef.h>
#include <Wbemidl.h>
#include <string>
#include "Windows.h"
#include "Utils.hpp"
#ifndef ONECORE
#pragma comment(lib, "wbemuuid.lib")
#endif

class SerialPort::serialport_impl
{
	bool closed_;
	HANDLE handle;
	OVERLAPPED writeOverlapped;
	OVERLAPPED readOverlapped;

public:
	serialport_impl()
	{
		closed_ = true;
		handle = NULL;
		memset(&writeOverlapped, 0, sizeof(OVERLAPPED));
		memset(&readOverlapped, 0, sizeof(OVERLAPPED));
	}
	~serialport_impl() {
		close();
	}

	bool isClosed() {
		return closed_;
	}

	int connect(const char* portName, int baudRate)
	{
		int dataBits = 8;
		Parity parity = Parity::Parity_None;
		StopBits sb = StopBits::StopBits_10;
		bool dtrEnable = false;
		bool rtsEnable = false;
		Handshake hs = Handshake::Handshake_None;
		int readTimeout = -1;
		int writeTimeout = -1;
		int readBufferSize = 8192;
		int writeBufferSize = 8192;

		std::string port = portName;
#ifndef ONECORE
		if (port.substr(0, 4) != "\\\\.\\")
		{
			port.insert(0, "\\\\.\\");
		}
#endif
        std::wstring wide(port.begin(), port.end());

		handle = CreateFileW(wide.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, 0);

		if (handle == INVALID_HANDLE_VALUE)
		{
			return GetLastError();
		}

		HRESULT hr = setAttributes(baudRate, parity, dataBits, sb, hs, readTimeout, writeTimeout);
		if (hr != 0)
		{
			return hr;
		}
		if (!PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR) || !SetupComm(handle, readBufferSize, writeBufferSize))
		{
			return GetLastError();
		}

		COMMTIMEOUTS timeouts;
		// FIXME: The windows api docs are not very clear about read timeouts,
		// and we have to simulate infinite with a big value (uint.MaxValue - 1)
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
		timeouts.ReadTotalTimeoutConstant = (readTimeout == -1 ? MAXDWORD - 1 : (DWORD)readTimeout);

		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = (writeTimeout == -1 ? MAXDWORD : (DWORD)writeTimeout);
		if (!SetCommTimeouts(handle, &timeouts))
		{
			return GetLastError();
		}

		// set signal
		DWORD dwFunc = (dtrEnable ? SETDTR : CLRDTR);
		if (!EscapeCommFunction(handle, dwFunc))
		{
			return GetLastError();
		}

		if (hs != Handshake_RequestToSend &&
			hs != Handshake_RequestToSendXonXoff)
		{
			dwFunc = (dtrEnable ? SETRTS : CLRRTS);
			if (!EscapeCommFunction(handle, dwFunc))
			{
				return GetLastError();
			}
		}

		writeOverlapped.Internal = 0;
		writeOverlapped.InternalHigh = 0;
		writeOverlapped.Offset = 0;
		writeOverlapped.OffsetHigh = 0;
		writeOverlapped.Pointer = 0;
		writeOverlapped.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

		if (writeOverlapped.hEvent == NULL)
		{
			return E_OUTOFMEMORY;
		}

		readOverlapped.Internal = 0;
		readOverlapped.InternalHigh = 0;
		readOverlapped.Offset = 0;
		readOverlapped.OffsetHigh = 0;
		readOverlapped.Pointer = 0;
		readOverlapped.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

		if (readOverlapped.hEvent == NULL)
		{
			return E_OUTOFMEMORY;
		}
		closed_ = false;
		return S_OK;
	}

	int setAttributes(int baudRate, Parity parity, int dataBits, StopBits bits, Handshake hs, int readTimeout, int writeTimeout)
	{
		DCB dcb;
		if (!GetCommState(this->handle, &dcb))
		{
			return GetLastError();
		}

		dcb.BaudRate = baudRate;
		dcb.Parity = (byte)parity;
		dcb.ByteSize = (byte)dataBits;

        switch (bits)
        {
        case StopBits_10:
            dcb.StopBits = ONESTOPBIT;
            break;
        case StopBits_15:
            dcb.StopBits = ONE5STOPBITS;
            break;
        case StopBits_20:
            dcb.StopBits = TWOSTOPBITS;
            break;
        default:
            break;
        }

		// Clear Handshake flags
		dcb.fOutxCtsFlow = 0;
		dcb.fOutX = 0;
		dcb.fInX = 0;
		dcb.fRtsControl = 0;

		// Set Handshake flags
		switch (hs)
		{
		case Handshake_None:
			break;
		case Handshake_XonXoff:
			dcb.fOutX = 1;
			dcb.fInX = 1;
			break;
		case Handshake_RequestToSend:
			dcb.fOutxCtsFlow = 1;
			dcb.fRtsControl = 1;
			break;
		case Handshake_RequestToSendXonXoff:
			dcb.fOutX = 1;
			dcb.fInX = 1;
			dcb.fOutxCtsFlow = 1;
			dcb.fRtsControl = 1;
			break;
		default: // Shouldn't happen
			break;
		}

		if (!SetCommState(handle, &dcb))
		{
			return GetLastError();
		}
		return S_OK;
	}

	int write(const uint8_t* ptr, int count)
	{
		BOOL rc = WriteFile(handle, ptr, count, NULL, &writeOverlapped);
		if (rc)
		{
			return S_OK;
		}
		HRESULT hr = GetLastError();
		if (hr != ERROR_IO_PENDING)
		{
			return -1;
		}

		DWORD bytesWritten = 0;
		if (!GetOverlappedResult(handle, &writeOverlapped, &bytesWritten, TRUE))
		{
			//return GetLastError();
			return -1;
		}

		if (bytesWritten != count)
		{
			return -1;
		}
		return count;
	}

	int read(uint8_t* buffer, int bytesToRead)
	{
		if (buffer == NULL)
		{
			return -1;
		}

		DWORD numberOfBytesRead = 0;
		if (ReadFile(handle, buffer, bytesToRead, &numberOfBytesRead, &readOverlapped))
		{
			return static_cast<int>(numberOfBytesRead);
		}

		HRESULT hr = GetLastError();
		if (hr != ERROR_IO_PENDING)
		{
			return -1;
		}

		if (!GetOverlappedResult(handle, &readOverlapped, &numberOfBytesRead, TRUE))
		{
			//return GetLastError();
			return -1;
		}

		return static_cast<int>(numberOfBytesRead);
	}

	void close()
	{
		closed_ = true;

		if (handle != 0)
		{
			CloseHandle(handle);
			handle = 0;
		}
		if (writeOverlapped.hEvent != 0)
		{
			CloseHandle(writeOverlapped.hEvent);
			writeOverlapped.hEvent = 0;
		}
		if (readOverlapped.hEvent != 0)
		{
			CloseHandle(writeOverlapped.hEvent);
			readOverlapped.hEvent = 0;
		}
	}

};

#else

#include <termios.h>
#include <unistd.h>
#include <fcntl.h> 
#include <errno.h>
#include <string.h>

class SerialPort::serialport_impl
{
	int fd;
	bool closed_;

public:
	serialport_impl() {
		closed_ = true;
		fd = -1;
	}
	~serialport_impl() {
		close();
	}

	bool isClosed() {
		return closed_;
	}

	int connect(const char* portName, int baudRate)
	{

		int dataBits = 8;
		Parity parity = Parity::Parity_None;
		StopBits sb = StopBits::StopBits_10;
		Handshake hs = Handshake::Handshake_None;
		int readTimeout = -1;
		int writeTimeout = -1;

		fd = open(portName, O_RDWR | O_NOCTTY);
		if (fd == -1)
		{
			return -1;
		}
		if (setAttributes(baudRate, parity, dataBits, sb, hs, readTimeout, writeTimeout) != 0)
			return -1;

		closed_ = false;
		return 0;
	}

	int write(const uint8_t* ptr, int count)
	{
		if (closed_) {
			return -1;
		}
		return ::write(fd, ptr, count);
	}

	int read(uint8_t* buffer, int bytesToRead)
	{
		if (closed_) {
			return -1;
		}
		return ::read(fd, buffer, bytesToRead);
	}

	void close()
	{
		closed_ = true;
		if (fd >= 0)
		{
			::close(fd);
			fd = -1;
		}
	}

	int setAttributes(int baudRate, Parity parity, int dataBits, StopBits stopBits, Handshake handshake, int readTimeout, int writeTimeout)
	{
        unused(writeTimeout);
		struct termios tty;
		::memset(&tty, 0, sizeof tty);
		if (tcgetattr(fd, &tty) != 0)
		{
			//Utils::logMessage("error %d from tcgetattr", errno);
			return -1;
		}

		int currentSpeed = getSpeed();
		if (currentSpeed != baudRate) {
			setSpeed(baudRate);
		}

		int size = CS8;
		switch (dataBits) {
		case 5:
			size = CS5;
			break;
		case 6:
			size = CS6;
			break;
		case 7:
			size = CS7;
			break;
		case 8:
			size = CS8;
			break;
		default:
			//Utils::logMessage("unsupported data size %d (expecting 5,6,7, or 8)", dataBits);
			return -1;
		}

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | size;     // data bits

		tty.c_iflag &= ~IGNBRK;         // disable break processing
		tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // no signaling chars, no echo, no canonical processing
		tty.c_oflag &= ~OPOST;
		tty.c_cc[VMIN] = 1;             // read blocks with a minimum of 1 char.
		if (readTimeout != -1)
		{
			tty.c_cc[VTIME] = readTimeout / 100;  // convert milliseconds to deciseconds.
		}
		tty.c_iflag &= ~(IXANY); // turn off IAXANY.

		if (handshake != Handshake_RequestToSendXonXoff)
		{
			tty.c_iflag &= ~(IXON | IXOFF); // shut off xon/xoff ctrl
		}

		tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
										// enable reading

		int parityFlags = 0;
		switch (parity) {
		case Parity_Odd:
			parityFlags = PARENB | PARODD;
			break;
		case Parity_Even:
			parityFlags = PARENB;
			break;
		case Parity_Mark:
#if defined(__APPLE__)
			// posix doesn't have mark/space parity.
			parityFlags = PARENB | PARODD;
#else
			parityFlags = PARENB | PARODD | CMSPAR;
#endif
			break;
		case Parity_Space:
#if defined(__APPLE__)
			// posix doesn't have mark/space parity.
			parityFlags = PARENB;
#else
			parityFlags = PARENB | CMSPAR;
#endif
			break;
		default:
			break;
		}
		if (parity != Parity_None) {
			tty.c_cflag &= ~(parityFlags);
		}

		int stopBitFlag = 0;
		switch (stopBits)
		{
		case StopBits_10:
			// this is the default.
			break;
		case StopBits_15:
			// not sure this is supported...
			break;
		case StopBits_20:
			stopBitFlag = CSTOPB;
			break;
		default:
			break;
		}
		if (stopBits != StopBits_None)
		{
			tty.c_cflag &= ~CSTOPB;
		}

		if (handshake == Handshake_RequestToSend) {
			tty.c_cflag &= ~CRTSCTS;
		}

		if (tcsetattr(fd, TCSANOW, &tty) != 0)
		{
			//Utils::logMessage("error %d from tcsetattr", errno);
			return -1;
		}
		return 0;

	}


	int getSpeed()
	{
		struct termios tty;
		memset(&tty, 0, sizeof tty);
		if (tcgetattr(fd, &tty) != 0)
		{
			//Utils::logMessage("error %d from tcgetattr", errno);
			return -1;
		}

		int value = 0;
		speed_t baud = ::cfgetospeed(&tty);
		switch (baud)
		{
			// First do those specified by POSIX.
		case B0: value = 0; break;
		case B50: value = 50; break;
		case B75: value = 75; break;
		case B110: value = 110; break;
		case B134: value = 134; break;
		case B150: value = 150; break;
		case B200: value = 200; break;
		case B300: value = 300; break;
		case B600: value = 600; break;
		case B1200: value = 1200; break;
		case B1800: value = 1800; break;
		case B2400: value = 2400; break;
		case B4800: value = 4800; break;
		case B9600: value = 9600; break;
		case B19200: value = 19200; break;
		case B38400: value = 38400; break;
			// Now conditionally handle a bunch of extended rates.
# ifdef B7200
		case B7200: value = 7200; break;
# endif
# ifdef B14400
		case B14400: value = 14400; break;
# endif
# ifdef B57600
		case B57600: value = 57600; break;
# endif
# ifdef B115200
		case B115200: value = 115200; break;
# endif
# ifdef B230400
		case B230400: value = 230400; break;
# endif
# ifdef B460800
		case B460800: value = 460800; break;
# endif
# ifdef B500000
		case B500000: value = 500000; break;
# endif
# ifdef B576000
		case B576000: value = 576000; break;
# endif
# ifdef B921600
		case B921600: value = 921600; break;
# endif
# ifdef B1000000
		case B1000000: value = 1000000; break;
# endif
# ifdef B1152000
		case B1152000: value = 1152000; break;
# endif
# ifdef B2000000
		case B2000000: value = 2000000; break;
# endif
# ifdef B3000000
		case B3000000: value = 3000000; break;
# endif
# ifdef B3500000
		case B3500000: value = 3500000; break;
# endif
# ifdef B4000000
		case B4000000: value = 4000000; break;
# endif
		default:
			value = 0;
			break;
		}
		return value;
	}

	void setSpeed(int baudRate)
	{
		speed_t value = B0;
		switch (baudRate)
		{
			// First do those specified by POSIX.
		case 0:     value = B0; break;
		case 50:    value = B50; break;
		case 75:    value = B75; break;
		case 110:   value = B110; break;
		case 134:   value = B134; break;
		case 150:   value = B150; break;
		case 200:   value = B200; break;
		case 300:   value = B300; break;
		case 600:   value = B600; break;
		case 1200:  value = B1200; break;
		case 1800:  value = B1800; break;
		case 2400:  value = B2400; break;
		case 4800:  value = B4800; break;
		case 9600:  value = B9600; break;
		case 19200: value = B19200; break;
		case 38400: value = B38400; break;
			// Now conditionally handle a bunch of extended rates.
# ifdef B7200
		case 7200: value = B7200; break;
# endif
# ifdef B14400
		case 14400: value = B14400; break;
# endif
# ifdef B57600
		case 57600: value = B57600; break;
# endif
# ifdef B115200
		case 115200: value = B115200; break;
# endif
# ifdef B230400
		case 230400: value = B230400; break;
# endif
# ifdef B460800
		case 460800: value = B460800; break;
# endif
# ifdef B500000
		case 500000: value = B500000; break;
# endif
# ifdef B576000
		case 576000: value = B576000; break;
# endif
# ifdef B921600
		case 921600: value = B921600; break;
# endif
# ifdef B1000000
		case 1000000: value = B1000000; break;
# endif
# ifdef B1152000
		case 1152000: value = B1152000; break;
# endif
# ifdef B2000000
		case 2000000: value = B2000000; break;
# endif
# ifdef B3000000
		case 3000000: value = B3000000; break;
# endif
# ifdef B3500000
		case 3500000: value = B3500000; break;
# endif
# ifdef B4000000
		case 4000000: value = B4000000; break;
# endif
		default:
			value = B0;
			break;
		}

		struct termios tty;
		memset(&tty, 0, sizeof tty);
		if (tcgetattr(fd, &tty) != 0)
		{
			//Utils::logMessage("error %d from tcgetattr", errno);
		}

		cfsetospeed(&tty, value);
		cfsetispeed(&tty, value);
	}
};

#endif


SerialPort::SerialPort()
{
	impl_.reset(new SerialPort::serialport_impl());
}


SerialPort::~SerialPort()
{
	impl_->close();
}

bool SerialPort::isClosed() {
	return impl_->isClosed();
}

int SerialPort::connect(const char* portName, int baudRate) {
	return impl_->connect(portName, baudRate);
}

int
SerialPort::write(const uint8_t* ptr, int count)
{
	return impl_->write(ptr, count);
}

int
SerialPort::read(uint8_t* buffer, int bytesToRead)
{
	return impl_->read(buffer, bytesToRead);
}

void
SerialPort::close()
{
	impl_->close();
}
