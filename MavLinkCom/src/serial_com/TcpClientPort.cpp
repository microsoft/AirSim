// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TcpClientPort.hpp"
#include "Utils.hpp"
#include <boost/lexical_cast.hpp>
using namespace common_utils;

// theoretical datagram max size, in practice the data link layer can impose additional limits.
#define TCP_MAXBUF_SIZE 65535

TcpClientPort::TcpClientPort() : socket_(io_service_) {
    closed_ = true;
    waiting_ = false;
    // increase send buffer to 1mb from default of 64kb
    boost::asio::socket_base::send_buffer_size option(1000000);

    read_buf_raw_ = new char[TCP_MAXBUF_SIZE];
    if (read_buf_raw_ == nullptr) {
        throw std::runtime_error("out of memory");
    }
}

TcpClientPort::~TcpClientPort() {
    close();
    delete read_buf_raw_;
    read_buf_raw_ = nullptr;
}

void TcpClientPort::close() {
    if (!closed_) {
        closed_ = true;
        socket_.close();
        if (waiting_) {
            waiting_ = false;
            available_.post();
        }
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }
    io_service_.stop();
}

void TcpClientPort::start() {
    // this is called from TcpServer when this ports sockets has been bound to a new accepted connection.

    //local_endpoint_ = socket_.local_endpoint();

    //remote_endpoint_ = socket_.remote_endpoint();

    closed_ = false;

    read_thread_ = std::thread{ &TcpClientPort::readPackets, this };
}

void TcpClientPort::connect(const std::string& localHost, int localPort, const std::string& remoteHost, int remotePort) {
    if (!socket_.is_open()) {
        socket_.open(tcp::v4());
    }

    tcp::resolver resolver(io_service_);
    {
        std::string portName = boost::lexical_cast<std::string>(localPort);
        tcp::resolver::query query(tcp::v4(), localHost, portName);
        tcp::resolver::iterator iter = resolver.resolve(query);
        local_endpoint_ = *iter;
    }

    if (remotePort == 0) {
        throw std::runtime_error("remote port cannot be zero");
    }

    std::string remotePortName = boost::lexical_cast<std::string>(remotePort);
    tcp::resolver::query query(tcp::v4(), remoteHost, remotePortName);
    tcp::resolver::iterator iter = resolver.resolve(query);
    remote_endpoint_ = *iter;

    socket_.connect(remote_endpoint_);

    closed_ = false;

    read_thread_ = std::thread{ &TcpClientPort::readPackets, this };

}

void TcpClientPort::readPackets() {
    socket_.async_receive(
        boost::asio::buffer(read_buf_raw_, TCP_MAXBUF_SIZE),
        boost::bind(
            &TcpClientPort::on_receive,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));

    io_service_.run();
}

int TcpClientPort::write(const uint8_t* ptr, int count) {
    if (closed_) {
        return 0;
    }
    size_t size = static_cast<size_t>(count);
    size_t sent = socket_.send(boost::asio::buffer(ptr, count));

    return count;
}

void TcpClientPort::on_receive(const boost::system::error_code& ec, size_t bytes_transferred) {
    boost::mutex::scoped_lock guard(mutex_);
    if (!socket_.is_open())
        return;

    if (ec) {
        printf("read tcp port %d error %d: %s\n", local_endpoint_.port(), ec.value(), ec.message().c_str());
        return;
    }

    if (bytes_transferred > 0) {
        sbuf_.sputn(read_buf_raw_, bytes_transferred);
        if (waiting_) {
            waiting_ = false;
            available_.post();
        }
    }

    // next packet...
    socket_.async_receive(
        boost::asio::buffer(read_buf_raw_, TCP_MAXBUF_SIZE),
        boost::bind(
            &TcpClientPort::on_receive,
            this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

int
TcpClientPort::read(uint8_t* buffer, int bytesToRead) {
    char* ptr = reinterpret_cast<char*>(buffer);
    int bytesread = 0;
    while (bytesread < bytesToRead && !closed_) {
        int delta = 0;
        int remaining = bytesToRead - bytesread;
        {
            boost::mutex::scoped_lock guard(mutex_);
            if (!socket_.is_open())
                return -1;
            delta = static_cast<int>(sbuf_.sgetn(ptr + bytesread, remaining));
            bytesread += delta;
        }
        if (delta == 0 && bytesread > 0) {
            // then return what we have.
            break;
        }
        if (delta == 0) {
            // we have exhausted available bytes, so time to wait for more
            waiting_ = true;
            available_.timed_wait(1000);
        }
    }
    return bytesread;
}
