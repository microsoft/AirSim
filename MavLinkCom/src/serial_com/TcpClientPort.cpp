// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Utils.hpp"
#include "TcpClientPort.hpp"
#include <stdio.h>
#include <string.h>
#include "SocketInit.hpp"
#include "wifi.h"

using namespace mavlink_utils;

#ifdef _WIN32
// windows
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

typedef int socklen_t;
static bool socket_initialized_ = false;
#else

// posix
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
typedef int SOCKET;
const int INVALID_SOCKET = -1;
const int ERROR_ACCESS_DENIED = EACCES;

inline int WSAGetLastError() {
	return errno;
}
const int SOCKET_ERROR = -1;
#define E_NOT_SUFFICIENT_BUFFER ENOMEM

#endif

class TcpClientPort::TcpSocketImpl
{
	SocketInit init;
	SOCKET sock = INVALID_SOCKET;
	sockaddr_in localaddr;
	sockaddr_in remoteaddr;
	bool closed_ = true;
public:

	bool isClosed() {
		return closed_;
	}

    int getRssi(const char* ifaceName)
    {
        return getWifiRssi(static_cast<int>(sock), ifaceName);
    }
	static void resolveAddress(const std::string& ipAddress, int port, sockaddr_in& addr)
	{
		struct addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);

		bool found = false;
		struct addrinfo *result = NULL;
		std::string serviceName = std::to_string(port);
		int rc = getaddrinfo(ipAddress.c_str(), serviceName.c_str(), &hints, &result);
		if (rc != 0) {
			throw std::runtime_error(Utils::stringf("TcpClientPort getaddrinfo failed with error: %d\n", rc));
		}
		for (struct addrinfo *ptr = result; ptr != NULL; ptr = ptr->ai_next)
		{
			if (ptr->ai_family == AF_INET && ptr->ai_socktype == SOCK_STREAM && ptr->ai_protocol == IPPROTO_TCP)
			{
				// found it!
				sockaddr_in* sptr = reinterpret_cast<sockaddr_in*>(ptr->ai_addr);
				addr.sin_family = sptr->sin_family;
				addr.sin_addr.s_addr = sptr->sin_addr.s_addr;
				addr.sin_port = sptr->sin_port;
				found = true;
				break;
			}
		}

		freeaddrinfo(result);
		if (!found) {
			throw std::runtime_error(Utils::stringf("TcpClientPort could not resolve ip address for '%s:%d'\n", ipAddress.c_str(), port));
		}
	}

	int connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort)
	{
		sock = socket(AF_INET, SOCK_STREAM, 0);

		resolveAddress(localHost, localPort, localaddr);
		resolveAddress(remoteHost, remotePort, remoteaddr);

		// bind socket to local address.
		socklen_t addrlen = sizeof(sockaddr_in);
		int rc = bind(sock, reinterpret_cast<sockaddr*>(&localaddr), addrlen);
		if (rc < 0)
		{
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("TcpClientPort socket bind failed with error: %d\n", hr));
		}

		rc = ::connect(sock, reinterpret_cast<sockaddr*>(&remoteaddr), addrlen);
		if (rc != 0) {
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("TcpClientPort socket connect failed with error: %d\n", hr));
		}

		closed_ = false;
		return 0;
	}

	void accept(const std::string& localHost, int localPort)
	{
		SOCKET local = socket(AF_INET, SOCK_STREAM, 0);

		resolveAddress(localHost, localPort, localaddr);

		// bind socket to local address.
		socklen_t addrlen = sizeof(sockaddr_in);
		int rc = ::bind(local, reinterpret_cast<sockaddr*>(&localaddr), addrlen);
		if (rc < 0)
		{
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("TcpClientPort socket bind failed with error: %d\n", hr));
		}

		// start listening for incoming connection
		rc = ::listen(local, 1);
		if (rc < 0)
		{
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("TcpClientPort socket listen failed with error: %d\n", hr));
		}

		// accept 1
		sock = ::accept(local, reinterpret_cast<sockaddr*>(&remoteaddr), &addrlen);
		if (sock == INVALID_SOCKET) {
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("TcpClientPort accept failed with error: %d\n", hr));
		}

#ifdef _WIN32
        // don't need to accept any more, so we can close this one.
        ::closesocket(local);
#else
		int fd = static_cast<int>(sock);
		::close(fd);
#endif

		closed_ = false;
	}

    void setNonBlocking()
    {
#ifdef _WIN32
        unsigned long mode = 1;
        int rc = ioctlsocket(sock, FIONBIO, &mode);
#else
        int fd = static_cast<int>(sock);
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags == -1) {
			throw std::runtime_error(Utils::stringf("fcntl failed with error: %d\n", errno));
		}
        flags |= O_NONBLOCK;
        int rc = fcntl(fd, F_SETFL, flags);
#endif
        if (rc != 0) {
#ifdef _WIN32
            rc = WSAGetLastError();
#endif
            throw std::runtime_error(Utils::stringf("TcpClientPort setNonBlocking failed with error: %d\n", rc));
        }
    }

    void setNoDelay()
    {
        int flags = 1;
#ifdef _WIN32
        int rc = ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&flags), sizeof(flags));
#else
        int fd = static_cast<int>(sock);
        int rc = ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&flags), sizeof(flags));
#endif

        if (rc != 0)
        {
#ifdef _WIN32
            rc = WSAGetLastError();
#endif
            throw std::runtime_error(Utils::stringf("TcpClientPort set TCP_NODELAY failed: %d\n", rc));
        }
    }

	// write to the serial port
	int write(const uint8_t* ptr, int count)
	{
		socklen_t addrlen = sizeof(sockaddr_in);
		int hr = send(sock, reinterpret_cast<const char*>(ptr), count, 0);
		if (hr == SOCKET_ERROR)
		{
			throw std::runtime_error(Utils::stringf("TcpClientPort socket send failed with error: %d\n", hr));
		}

		return hr;
	}

	int read(uint8_t* result, int bytesToRead)
	{
		int bytesRead = 0;
		// try and receive something, up until port is closed anyway.

		while (!closed_)
		{
			socklen_t addrlen = sizeof(sockaddr_in);
			int rc = recv(sock, reinterpret_cast<char*>(result), bytesToRead, 0);
			if (rc < 0)
			{
#ifdef _WIN32
				int hr = WSAGetLastError();
				if (hr == WSAEMSGSIZE)
				{
					// message was too large for the buffer, no problem, return what we have.
				}
				else if (hr == WSAECONNRESET || hr == ERROR_IO_PENDING)
				{
					// try again - this can happen if server recreates the socket on their side.
					continue;
				}
				else
#else
				int hr = errno;
				if (hr == EINTR)
				{
					// skip this, it is was interrupted.
					continue;
				}
				else
#endif
				{
					return -1;
				}
			}

			if (rc == 0)
			{
				//printf("Connection closed\n");
				return -1;
			}
			else
			{
				return rc;
			}
		}
		return -1;
	}


	void close()
	{
		if (!closed_) {
			closed_ = true;

#ifdef _WIN32
			closesocket(sock);
#else
			int fd = static_cast<int>(sock);
			::close(fd);
#endif
		}
	}

	std::string remoteAddress() {
		return inet_ntoa(remoteaddr.sin_addr);
	}

	int remotePort() {
		return ntohs(remoteaddr.sin_port);
	}

};

//-----------------------------------------------------------------------------------------

TcpClientPort::TcpClientPort()
{
	impl_.reset(new TcpSocketImpl());
}

TcpClientPort::~TcpClientPort()
{
	close();
}

void TcpClientPort::close()
{
	impl_->close();
}

void TcpClientPort::connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort)
{
	impl_->connect(localHost, localPort, remoteHost, remotePort);
}

void TcpClientPort::accept(const std::string& localHost, int localPort)
{
	impl_->accept(localHost, localPort);
}

int TcpClientPort::write(const uint8_t* ptr, int count)
{
	return impl_->write(ptr, count);
}

int
TcpClientPort::read(uint8_t* buffer, int bytesToRead)
{
	return impl_->read(buffer, bytesToRead);
}

bool TcpClientPort::isClosed()
{
	return impl_->isClosed();
}

std::string TcpClientPort::remoteAddress()
{
	return impl_->remoteAddress();
}

int TcpClientPort::remotePort()
{
	return impl_->remotePort();
}

int TcpClientPort::getRssi(const char* ifaceName)
{
    return impl_->getRssi(ifaceName);
}

void TcpClientPort::setNoDelay()
{
    impl_->setNoDelay();
}

void TcpClientPort::setNonBlocking()
{
    impl_->setNonBlocking();
}
