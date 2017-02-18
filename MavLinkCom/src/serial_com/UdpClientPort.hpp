// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SERIAL_COM_UDPCLIENTPORT_HPP
#define SERIAL_COM_UDPCLIENTPORT_HPP

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <stdio.h>
#include <thread>
#include "Port.h"
#include "MavLinkSemaphore.hpp"

using boost::asio::ip::udp;

class UdpClientPort : public Port
{
public:
	UdpClientPort();
	
	// Connect can set you up two different ways.  Pass 0 for local port to get any free local
	// port and pass a fixed remotePort if you want to send to a specific remote port. 
	// Conversely, pass a fix local port to bind to, and 0 for remotePort if you want to 
	// allow any remote sender to send to your specific local port.  localHost allows you to
	// be specific about local adapter ip address, pass "127.0.0.1" if you don't care.
	void connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort);

	~UdpClientPort();

	// write to the port, return number of bytes written or -1 if error.
	virtual int write(const uint8_t* ptr, int count);

	// read a given number of bytes from the port (blocking until the requested bytes are available).
	// return the number of bytes read or -1 if error.
	virtual int read(uint8_t* buffer, int bytesToRead);

	// close the port.
	virtual void close();

	virtual bool isClosed() {
		return closed;
	}
private:
	void readPackets();
	void on_receive(const boost::system::error_code& ec, size_t bytes_transferred);
	boost::asio::streambuf sbuf;
	boost::asio::io_service io_service;
	udp::socket socket;
	std::thread read_thread;
	udp::endpoint remote_endpoint;
	udp::endpoint local_endpoint;
	boost::mutex mutex;
	mavlinkcom::MavLinkSemaphore available;
	char* read_buf_raw;
	bool closed;
};

#endif
