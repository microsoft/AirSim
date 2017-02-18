// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_MavLinkTcpServerImpl_hpp
#define MavLinkCom_MavLinkTcpServerImpl_hpp

#include <memory>
#include <vector>
#include <thread>
#include <string>
#include "MavLinkTcpServer.hpp"

using namespace mavlinkcom;

class TcpServer;

namespace mavlinkcom_impl
{
	class MavLinkTcpServerImpl
	{
	public:
		MavLinkTcpServerImpl(const std::string& local_addr, int local_port);
		~MavLinkTcpServerImpl();

		void acceptTcp(const std::string& nodeName, MavLinkConnectionHandler handler);
	private:

		std::string local_address_;
		int local_port_;
		MavLinkConnectionHandler handler_;
		std::string accept_node_name_;
		std::shared_ptr<TcpServer> server_;
	};
}

#endif
