// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SERIAL_COM_UDPSERVER_HPP
#define SERIAL_COM_UDPSERVER_HPP

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <stdio.h>
using boost::asio::ip::udp;

class UdpServer;

typedef std::function<void(const UdpServer& port, udp::endpoint& sender, const char* msg, int length)> UdpServerMessageHandler;

// This class listens on a given local port for messages from any remote macine.
// The UdpServerMessageHandler is called with each packet received passing the 
// endpoint information of the sender (which chould be different fro each packet)
// It is up to the user then to decide if and what to send back using sendResponse.
class UdpServer
{
public:
	UdpServer(boost::asio::io_service& service, short port)
		: io_service(service),
		socket(service, udp::endpoint(udp::v4(), port))
	{
	}

	void start(UdpServerMessageHandler callback)
	{
		this->handler = callback;
		socket.async_receive_from(
			boost::asio::buffer(data, max_length), sender_endpoint,
			boost::bind(&UdpServer::handle_receive_from, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

	void sendResponse(udp::endpoint& remote, const char* buffer, int len)
	{
		socket.async_send_to(
			boost::asio::buffer(buffer, len), remote,
			boost::bind(&UdpServer::handle_send_to, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

private:
	void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd)
	{
		if (!error && bytes_recvd > 0)
		{
			handler(*this, sender_endpoint, data, static_cast<int>(bytes_recvd));
		}

		// start next msg
		socket.async_receive_from(
			boost::asio::buffer(data, max_length), sender_endpoint,
			boost::bind(&UdpServer::handle_receive_from, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

	void handle_send_to(const boost::system::error_code& error, size_t bytes_sent)
	{
		socket.async_receive_from(
			boost::asio::buffer(data, max_length), sender_endpoint,
			boost::bind(&UdpServer::handle_receive_from, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

private:
	UdpServerMessageHandler handler;
	boost::asio::io_service& io_service;
	udp::socket socket;
	udp::endpoint sender_endpoint;
	enum { max_length = 1024 };
	char data[max_length];
};

#endif
