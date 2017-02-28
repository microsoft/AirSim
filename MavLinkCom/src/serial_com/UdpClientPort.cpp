// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Utils.hpp"
#include "ThreadUtils.hpp"
#include "UdpClientPort.hpp"
#include <boost/lexical_cast.hpp>
using namespace common_utils;

// theoretical datagram max size, in practice the data link layer can impose additional limits.
#define UDP_MAXBUF_SIZE 65535

UdpClientPort::UdpClientPort() : socket(io_service, { udp::v4() })
{
	closed = true; 
	// increase send buffer to 1mb from default of 64kb
	boost::asio::socket_base::send_buffer_size option(1000000);
	
	read_buf_raw = new char[UDP_MAXBUF_SIZE];
	if (read_buf_raw == nullptr)
	{
		throw std::runtime_error("out of memory");
	}
}

UdpClientPort::~UdpClientPort()
{
	close();
	delete[] read_buf_raw;
	read_buf_raw = nullptr;
}

void UdpClientPort::close() 
{
	closed = true;
	socket.close();
	available.post(); 
	io_service.stop();
	if (read_thread.joinable())
	{
		read_thread.join();
	}
}

// Connect can set you up two different ways.  Pass 0 for local port to get any free local
// port and pass a fixed remotePort if you want to send to a specific remote port. 
// Conversely, pass a fix local port to bind to, and 0 for remotePort if you want to 
// allow any remote sender to send to your specific local port.  localHost allows you to
// be specific about local adapter ip address, pass "127.0.0.1" if you don't care.
void UdpClientPort::connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort)
{
	udp::resolver resolver(io_service);
	{
		std::string portName = boost::lexical_cast<std::string>(localPort);
		udp::resolver::query query(udp::v4(), localHost, portName);
		udp::resolver::iterator iter = resolver.resolve(query);
		local_endpoint = *iter;
	}

	if (remotePort != 0)
	{
		std::string remotePortName = boost::lexical_cast<std::string>(remotePort);
		udp::resolver::query query(udp::v4(), remoteHost, remotePortName);
		udp::resolver::iterator iter = resolver.resolve(query);
		remote_endpoint = *iter;
	}
	
	socket.bind(local_endpoint);

	closed = false;

	read_thread = std::thread{ &UdpClientPort::readPackets, this };

}

void UdpClientPort::readPackets()
{
	CurrentThread::setMaximumPriority();
	while (!closed) {
		boost::system::error_code ec;
		size_t bytes = socket.receive_from(
				boost::asio::buffer(read_buf_raw, UDP_MAXBUF_SIZE), remote_endpoint, 0, ec);			
		on_receive(ec, bytes);
	}

	// async mode is not used because boost will not run that async thread at maximum priority
	// which is what we need in the Unreal environment.
	//socket.async_receive_from(
	//	boost::asio::buffer(read_buf_raw, UDP_MAXBUF_SIZE), remote_endpoint,
	//	boost::bind(
	//		&UdpClientPort::on_receive,
	//		this, boost::asio::placeholders::error,
	//		boost::asio::placeholders::bytes_transferred));
	//io_service.run();
}

int UdpClientPort::write(const uint8_t* ptr, int count)
{
	if (closed) {
		return 0;
	}

	size_t size = static_cast<size_t>(count);
	if (remote_endpoint.port() != 0) {
		size_t sent = socket.send_to(boost::asio::buffer(ptr, count), remote_endpoint);
	}
	else {
		// todo: should we tell the user this packet was dropped?
	}
	return count;
}

void UdpClientPort::on_receive(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock guard(mutex);
	if (!socket.is_open())
		return;

	if (ec) {
		//printf("read udp port %d error %d\n", local_endpoint.port(), ec.value());
		// make sure we don't spin too badly on error retry conditions.
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	bool empty = sbuf.size() == 0;
	if (bytes_transferred > 0) {
		sbuf.sputn(read_buf_raw, bytes_transferred);
		if (empty) {
			available.post();
		}
	}

	// next packet...
	socket.async_receive_from(
		boost::asio::buffer(read_buf_raw, UDP_MAXBUF_SIZE), remote_endpoint,
		boost::bind(
			&UdpClientPort::on_receive,
			this, boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

int
UdpClientPort::read(uint8_t* buffer, int bytesToRead)
{
	char* ptr = reinterpret_cast<char*>(buffer);
	int bytesread = 0;
	while (bytesread < bytesToRead && !closed)
	{
		int delta = 0;
		int remaining = bytesToRead - bytesread;
		{
			boost::mutex::scoped_lock guard(mutex);
			if (!socket.is_open())
				return -1;
			delta = static_cast<int>(sbuf.sgetn(ptr + bytesread, remaining));
			bytesread += delta;
		}
		if (delta == 0 && bytesread > 0) {
			// then return what we have.
			break;
		}
		if (delta == 0) {
			// we have exhausted available bytes, so time to wait for more
			available.timed_wait(1000);
		}
	}
	return bytesread;
}
