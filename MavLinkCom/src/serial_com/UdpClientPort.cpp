// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Utils.hpp"
#include "ThreadUtils.hpp"
#include "UdpClientPort.hpp"
#include <stdio.h>
#include <string.h>

using namespace common_utils;

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
#include <cerrno>
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

class UdpClientPort::UdpSocketImpl
{
	SOCKET sock = INVALID_SOCKET;
	sockaddr_in localaddr;
	sockaddr_in remoteaddr;
	bool hasRemote = false;
	SOCKET sendSocket = INVALID_SOCKET;
	bool closed_ = true;
public:

	bool isClosed() {
		return closed_;
	}

	void initialize()
	{
#ifdef _WIN32
		if (!socket_initialized_) {
			socket_initialized_ = true;
			WSADATA wsaData;
			// Initialize Winsock
			int rc = WSAStartup(MAKEWORD(2, 2), (LPWSADATA)&wsaData);
			if (rc != 0) {
				throw std::runtime_error(Utils::stringf("UdpClientPort WSAStartup failed with error : %d\n", rc));
			}
		}
#endif
	}

	static void resolveAddress(const std::string& ipAddress, int port, sockaddr_in& addr)
	{
		struct addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_DGRAM;
		hints.ai_protocol = IPPROTO_UDP;

		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);

		bool found = false;
		struct addrinfo *result = NULL;
		std::string serviceName = std::to_string(port);
		int rc = getaddrinfo(ipAddress.c_str(), serviceName.c_str(), &hints, &result);
		if (rc != 0) {
			throw std::runtime_error(Utils::stringf("UdpClientPort getaddrinfo failed with error: %d\n", rc));
		}
		for (struct addrinfo *ptr = result; ptr != NULL; ptr = ptr->ai_next)
		{
			if (ptr->ai_family == AF_INET && ptr->ai_socktype == SOCK_DGRAM && ptr->ai_protocol == IPPROTO_UDP)
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

			throw std::runtime_error(Utils::stringf("UdpClientPort could not resolve ip address for '%s:%d'\n", ipAddress.c_str(), port));
		}
	}

	int connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort)
	{
		initialize();

		sock = socket(AF_INET, SOCK_DGRAM, 0);

		resolveAddress(localHost, localPort, localaddr);

		if (remoteHost != "") {
			hasRemote = true;
			resolveAddress(remoteHost, remotePort, remoteaddr);
		}
		else 
		{
			remoteaddr.sin_port = 0;
		}

		// bind socket to local address.
		socklen_t addrlen = sizeof(sockaddr_in);
		int rc = bind(sock, reinterpret_cast<sockaddr*>(&localaddr), addrlen);
		if (rc < 0)
		{
			int hr = WSAGetLastError();
			throw std::runtime_error(Utils::stringf("UdpClientPort connect bind failed with error: %d\n", hr));
			return hr;
		}
		closed_ = false;
		return 0;
	}

	// write to the serial port
	int write(const uint8_t* ptr, int count)
	{
		if (sendSocket == INVALID_SOCKET)
		{
			if (remoteaddr.sin_port == 0)
			{
				throw std::runtime_error("UdpClientPort cannot send until we've received something first so we can find out what port to send to.\n");
				return 0;
			}
		}

		socklen_t addrlen = sizeof(sockaddr_in);
		int hr = sendto(sock, reinterpret_cast<const char*>(ptr), count, 0, reinterpret_cast<sockaddr*>(&remoteaddr), addrlen);
		if (hr == SOCKET_ERROR)
		{
			throw std::runtime_error(Utils::stringf("UdpClientPort socket send failed with error: %d\n", hr));
		}

		return hr;
	}

	int read(uint8_t* result, int bytesToRead)
	{
		sockaddr_in other;

		int bytesRead = 0;
		// try and receive something, up until port is closed anyway.
		
		while (!closed_)
		{
			socklen_t addrlen = sizeof(sockaddr_in);
			int rc = recvfrom(sock, reinterpret_cast<char*>(result), bytesToRead, 0, reinterpret_cast<sockaddr*>(&other), &addrlen);
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
					//printf("#### recv failed with error: %d\n", hr);
					return -1;
				}
			}

			if (remoteaddr.sin_port == 0)
			{
				// we now have it.
				remoteaddr.sin_port = ntohs(other.sin_port);
			}
			else if (other.sin_addr.s_addr != remoteaddr.sin_addr.s_addr)
			{
				// this is from someone we are not interested in.
				continue;
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
};

//-----------------------------------------------------------------------------------------

UdpClientPort::UdpClientPort() 
{
	impl_.reset(new UdpSocketImpl());
}

UdpClientPort::~UdpClientPort()
{
	close();
}

void UdpClientPort::close() 
{
	impl_->close();
}

void UdpClientPort::connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort)
{
	impl_->connect(localHost, localPort, remoteHost, remotePort);
}

int UdpClientPort::write(const uint8_t* ptr, int count)
{
	return impl_->write(ptr, count);
}

int
UdpClientPort::read(uint8_t* buffer, int bytesToRead)
{
	return impl_->read(buffer, bytesToRead);
}

bool UdpClientPort::isClosed()
{
	return impl_->isClosed();
}