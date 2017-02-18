// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkTcpServerImpl.hpp"
#include "../serial_com/TcpServer.hpp"
#include "../serial_com/TcpClientPort.hpp"

using namespace mavlinkcom_impl;

MavLinkTcpServerImpl::MavLinkTcpServerImpl(const std::string& local_addr, int local_port) {
    local_address_ = local_addr;
    local_port_ = local_port;
    server_ = std::make_shared<TcpServer>(local_address_, local_port_);
}

MavLinkTcpServerImpl::~MavLinkTcpServerImpl() {
}

void MavLinkTcpServerImpl::acceptTcp(const std::string& nodeName, MavLinkConnectionHandler handler) {
    handler_ = handler;
    accept_node_name_ = nodeName;

    server_->accept([=](std::shared_ptr<TcpClientPort> port) {
        std::shared_ptr<MavLinkConnection> con = std::make_shared<MavLinkConnection>();
        con->startListening(accept_node_name_, port);
        handler_(con);
    });
}
