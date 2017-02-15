// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkTcpServer.hpp"
#include "impl/MavLinkTcpServerImpl.hpp"

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

MavLinkTcpServer::MavLinkTcpServer(const std::string& local_addr, int local_port)
	: impl_(new MavLinkTcpServerImpl(local_addr, local_port))
{
}

void MavLinkTcpServer::acceptTcp(const std::string& nodeName, MavLinkConnectionHandler handler)
{
	impl_->acceptTcp(nodeName, handler);
}

MavLinkTcpServer::MavLinkTcpServer() {
}
MavLinkTcpServer::~MavLinkTcpServer() {
}
//MavLinkTcpServer::MavLinkTcpServer(MavLinkTcpServer&&) = default;
//MavLinkTcpServer& MavLinkTcpServer::operator=(MavLinkTcpServer&&) = default;

