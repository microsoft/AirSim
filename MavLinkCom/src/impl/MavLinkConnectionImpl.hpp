// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_MavLinkConnectionImpl_hpp
#define MavLinkCom_MavLinkConnectionImpl_hpp

#include <memory>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include "MavLinkConnection.hpp"
#include "MavLinkMessageBase.hpp"
#include "MavLinkSemaphore.hpp"
#include "../serial_com/TcpServer.hpp"

using namespace mavlinkcom;

namespace mavlinkcom_impl {

	class MavLinkConnectionImpl
	{
	public:
		MavLinkConnectionImpl();
		static std::vector<SerialPortInfo> findSerialPorts(int vid, int pid);
		static std::shared_ptr<MavLinkConnection>  connectSerial(const std::string& nodeName, std::string portName, int baudrate = 115200, const std::string initString = "");
		static std::shared_ptr<MavLinkConnection>  connectLocalUdp(const std::string& nodeName, std::string localAddr, int localPort);
		static std::shared_ptr<MavLinkConnection>  connectRemoteUdp(const std::string& nodeName, std::string localAddr, std::string remoteAddr, int remotePort);
		static std::shared_ptr<MavLinkConnection>  connectTcp(const std::string& nodeName, std::string localAddr, const std::string& remoteIpAddr, int remotePort);

		std::string getName();
		int getTargetComponentId();
		int getTargetSystemId();
		~MavLinkConnectionImpl();
		void startListening(std::shared_ptr<MavLinkConnection> parent, const std::string& nodeName, std::shared_ptr<Port>  connectedPort);
		void startLoggingSendMessage(std::shared_ptr<MavLinkLog> log);
		void stopLoggingSendMessage();
		void close();
        bool isOpen();
		void sendMessage(const MavLinkMessageBase& msg);
		void sendMessage(const MavLinkMessage& msg);
		int subscribe(MessageHandler handler);
		void unsubscribe(int id);		
		uint8_t getNextSequence();
		void join(std::shared_ptr<MavLinkConnection> remote, bool subscribeToLeft = true, bool subscribeToRight = true);
	private:
		static std::shared_ptr<MavLinkConnection> createConnection(const std::string& nodeName, Port* port);
        void joinLeftSubscriber(std::shared_ptr<MavLinkConnection> remote, std::shared_ptr<MavLinkConnection>con, const MavLinkMessage& msg);
        void joinRightSubscriber(std::shared_ptr<MavLinkConnection>con, const MavLinkMessage& msg);
		void publishPackets();
		void readPackets();
		void drainQueue();
		std::string name;
		std::shared_ptr<Port> port;
		std::shared_ptr<MavLinkConnection> con_;
		int other_system_id = -1;
		int other_component_id = 0;
		uint8_t next_seq = 0;
		std::thread read_thread;
		std::string accept_node_name_;
		std::shared_ptr<TcpServer> server_; 
		std::shared_ptr<MavLinkLog> sendLog_;

		struct MessageHandlerEntry {
		public:
			int id;
			MessageHandler handler;
		};
		std::vector<MessageHandlerEntry> listeners;
		std::vector<MessageHandlerEntry> snapshot;
		bool snapshot_stale;
		std::mutex listener_mutex;
		uint8_t message_buf[300];
		std::mutex buffer_mutex;
		bool closed;
		std::thread publish_thread_;
		std::queue<MavLinkMessage> msg_queue_;
		std::mutex msg_queue_mutex_;
		MavLinkSemaphore msg_available_;
		bool waiting_for_msg_ = false;
		size_t max_queue_length_ = 0;
		long mavlink_errors_ = 0;
	};
}

#endif
