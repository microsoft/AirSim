// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "SerialPort.hpp"
#include <limits>
#include "Utils.hpp"

using namespace common_utils;
using namespace boost::asio;

#define SERIAL_PORT_READ_BUF_SIZE 65536

SerialPort::SerialPort() : port(io_service)
{
	closed_ = true;
	read_buf_raw = new char[SERIAL_PORT_READ_BUF_SIZE];
	if (read_buf_raw == nullptr)
	{
		throw std::runtime_error("out of memory");
	}
}


SerialPort::~SerialPort()
{
    close();
	delete read_buf_raw;
	read_buf_raw = nullptr;
}

int
SerialPort::connect(const char* portName, int baudRate)
{
	boost::system::error_code ec;

	port.open(portName, ec);
	if (ec) {
		printf("error : port.open() failed...com_port_name=%s, error=%s\n", portName, ec.message().c_str());
		return -1;
	}
	closed_ = false;
	// option settings...(rarely need to change these which is why these are not exposed).
	port.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
	port.set_option(boost::asio::serial_port_base::character_size(8));
	port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	
	read_thread = std::thread{ &SerialPort::readPackets, this };
	return 0;
}


void SerialPort::readPackets()
{
	port.async_read_some(
		boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&SerialPort::on_receive,
			this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));

	io_service.run();
}


void SerialPort::on_receive(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock guard(mutex);
	if (!port.is_open()) 
		return;

    if (ec && ec.value() != 2 /* end of file*/) {
        printf("read serial port error %d: %s\n", ec.value(), ec.message().c_str());
		return;
	}

	// buffer the bytes until someone calls read.
	if (bytes_transferred > 0) {
		bool empty = sbuf.size() == 0;
		sbuf.sputn(read_buf_raw, bytes_transferred);
		if (empty) {
			available.post();
		}
	}

	// read next
	port.async_read_some(
		boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&SerialPort::on_receive,
			this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));

}

int
SerialPort::write(const uint8_t* ptr, int count)
{
	if (!port.is_open())
		return -1;

	//port.async_write_some(boost::asio::buffer(ptr, count),
	//	boost::bind(
	//		&SerialPort::on_sent,
	//		this, boost::asio::placeholders::error,
	//		boost::asio::placeholders::bytes_transferred));

	//if (!written.timed_wait(10000)) {
	//	throw std::runtime_error("timeout trying to write to serial port");
	//}
	//return count;

    size_t write_count = boost::asio::write(port, boost::asio::buffer(ptr, count));
    return static_cast<int>(write_count);
}

void SerialPort::on_sent(const boost::system::error_code& ec, size_t bytes_transferred)
{   
    boost::mutex::scoped_lock guard(mutex);
	written.post();

	if (!port.is_open()) return;
	
	if (ec) {
		printf("write serial port error %d\n", ec.value());
		return;
	}
}

int
SerialPort::read(uint8_t* buffer, int bytesToRead)
{
	char* ptr = reinterpret_cast<char*>(buffer);
	int actualRead = 0;
	while (actualRead < bytesToRead)
	{
		int delta = 0;
		int remaining = bytesToRead - actualRead;
		{
			boost::mutex::scoped_lock guard(mutex);
			if (!port.is_open()) 
				return -1;
			delta = static_cast<int>(sbuf.sgetn(ptr + actualRead, remaining));
			actualRead += delta;
		}
		if (delta == 0 && actualRead > 0) {
			// then return what we have.
			break;
		}
		if (delta == 0) {
			// we have exhausted available bytes, so time to wait for more
			available.timed_wait(1000);
		}
	}
	return actualRead;
}

void
SerialPort::close()
{
	port.close();
	available.post();
	if (read_thread.joinable())
	{
		read_thread.join();
	}
	closed_ = true;
}

