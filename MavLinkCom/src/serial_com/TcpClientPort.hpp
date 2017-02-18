// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SERIAL_COM_TCPCLIENTPORT_HPP
#define SERIAL_COM_TCPCLIENTPORT_HPP

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <stdio.h>
#include <thread>
#include "Port.h"
#include "MavLinkSemaphore.hpp"

using boost::asio::ip::tcp;

class TcpClientPort : public Port {
  public:
    TcpClientPort();

    void connect(const std::string& localAddress, int localPort, const std::string& remoteAddress, int remotePort);

    ~TcpClientPort();

    // write to the port, return number of bytes written or -1 if error.
    virtual int write(const uint8_t* ptr, int count);

    // read a given number of bytes from the port (blocking until the requested bytes are available).
    // return the number of bytes read or -1 if error.
    virtual int read(uint8_t* buffer, int bytesToRead);

    // close the port.
    virtual void close();

    virtual bool isClosed() {
        return closed_;
    }

    // for use by TcpServer only.
    void start();
    tcp::socket& socket() {
        return socket_;
    }
  private:
    void readPackets();
    void on_receive(const boost::system::error_code& ec, size_t bytes_transferred);
    boost::asio::streambuf sbuf_;
    boost::asio::io_service io_service_;
    tcp::socket socket_;
    std::thread read_thread_;
    tcp::endpoint remote_endpoint_;
    tcp::endpoint local_endpoint_;
    boost::mutex mutex_;
    mavlinkcom::MavLinkSemaphore available_;
    bool waiting_;
    char* read_buf_raw_;
    bool closed_;
};

#endif
