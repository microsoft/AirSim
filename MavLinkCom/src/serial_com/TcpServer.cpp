// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "TcpServer.hpp"
#include "TcpClientPort.hpp"
using boost::asio::ip::tcp;

TcpServer::TcpServer(const std::string& localAddr, int localPort)
    : acceptor_(io_service_, tcp::endpoint(tcp::v4(), localPort)) {
    acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
    closed_ = false;
    worker_running_ = false;
}

void TcpServer::accept(ConnectionHandler handler) {
    if (closed_) {
        throw std::runtime_error("TcpServer is closed, so you cannot call start.");
    }
    this->handler_ = handler;

    std::shared_ptr<TcpClientPort> pointer = std::make_shared<TcpClientPort>();

    acceptor_.async_accept(pointer->socket(),
                           boost::bind(&TcpServer::handleAccept, this, pointer,
                                       boost::asio::placeholders::error));

    if (!worker_running_) {
        worker_running_ = true;
        worker_thread_ = std::thread{ &TcpServer::run, this };
    }
}

void TcpServer::run() {

    io_service_.run();
    worker_running_ = false;
}

void TcpServer::handleAccept(std::shared_ptr<TcpClientPort> newConnection, const boost::system::error_code& error) {
    if (!error) {
        if (!closed_) {
            newConnection->start();
            if (this->handler_ != nullptr) {
                this->handler_(newConnection);
            }
        }
    } else {
        printf("Error accepting socket %s\n", error.message().c_str());
    }
}

TcpServer::~TcpServer() {
    close();
}

void TcpServer::close() {
    if (!closed_) {
        closed_ = true;
        io_service_.stop();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }
}
