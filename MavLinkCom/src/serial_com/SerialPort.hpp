// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SERIAL_COM_SERIALPORT_HPP
#define SERIAL_COM_SERIALPORT_HPP

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <functional>
#include "MavLinkSemaphore.hpp"
#include "Port.h"


class SerialPort;

class SerialPort : public Port {
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
    void on_receive(const boost::system::error_code& ec, size_t bytes_transferred);
    void on_sent(const boost::system::error_code& ec, size_t bytes_transferred);
    void readPackets();

    bool closed_;
    boost::asio::streambuf sbuf;
    boost::asio::io_service io_service;
    boost::asio::serial_port port;
    std::thread read_thread;
    boost::mutex mutex;
    char* read_buf_raw;
    mavlinkcom::MavLinkSemaphore written;
    mavlinkcom::MavLinkSemaphore available;
};

#endif
