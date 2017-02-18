// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SERIAL_COM_TCPSERVER_HPP
#define SERIAL_COM_TCPSERVER_HPP

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <stdio.h>
#include <thread>
#include <functional>
#include <string>
#include "Port.h"
#include "TcpClientPort.hpp"

using boost::asio::ip::tcp;

typedef std::function<void(std::shared_ptr<TcpClientPort> port)> ConnectionHandler;

class TcpServer 
{
public:
	TcpServer(const std::string& localAddr, int localPort);

	// This method accepts one new connection
	void accept(ConnectionHandler handler);

	~TcpServer();

	// close the server
	void close();

	int port() { return acceptor_.local_endpoint().port(); }
	std::string localAddress() { return acceptor_.local_endpoint().address().to_string(); }

private:
	void handleAccept(std::shared_ptr<TcpClientPort> newSocket, const boost::system::error_code& error);
	void run();
	boost::asio::io_service io_service_;
	tcp::acceptor acceptor_;
	bool closed_;
	std::thread worker_thread_;
	bool worker_running_;
	ConnectionHandler handler_;
};

#endif
