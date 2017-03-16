// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Utils.hpp"
#include "ThreadUtils.hpp"
#include "MavLinkConnectionImpl.hpp"
#include "../serial_com/Port.h"
#include "../serial_com/SerialPort.hpp"
#include "../serial_com/UdpClientPort.hpp"
#include "../serial_com/TcpClientPort.hpp"
#define MAVLINK_PACKED

STRICT_MODE_OFF
#include "../mavlink/common/mavlink.h"
#include "../mavlink/mavlink_helpers.h"
#include "../mavlink/mavlink_types.h"
STRICT_MODE_ON

static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;

using namespace mavlink_utils;
using namespace mavlinkcom_impl;

MavLinkConnectionImpl::MavLinkConnectionImpl()
{
	// add our custom telemetry message length.
	telemetry_.crcErrors = 0;
	telemetry_.handlerMicroseconds = 0;
	telemetry_.messagesHandled = 0;
	telemetry_.messagesReceived = 0;
	telemetry_.messagesSent = 0;
	telemetry_.renderTime = 0;
	closed = true;
}
std::string MavLinkConnectionImpl::getName() {
	return name;
}

MavLinkConnectionImpl::~MavLinkConnectionImpl()
{
	con_.reset();
	close();
}

std::shared_ptr<MavLinkConnection>  MavLinkConnectionImpl::createConnection(const std::string& nodeName, std::shared_ptr<Port> port)
{
	// std::shared_ptr<MavLinkCom> owner, const std::string& nodeName
	std::shared_ptr<MavLinkConnection> con = std::make_shared<MavLinkConnection>();
	con->startListening(nodeName, port);
	return con;
}

std::shared_ptr<MavLinkConnection>  MavLinkConnectionImpl::connectLocalUdp(const std::string& nodeName, std::string localAddr, int localPort)
{
	std::shared_ptr<UdpClientPort> socket = std::make_shared<UdpClientPort>();

	socket->connect(localAddr, localPort, "", 0);

	return createConnection(nodeName, socket);
}

std::shared_ptr<MavLinkConnection>  MavLinkConnectionImpl::connectRemoteUdp(const std::string& nodeName, std::string localAddr, std::string remoteAddr, int remotePort)
{
	std::string local = localAddr;
	// just a little sanity check on the local address, if remoteAddr is localhost then localAddr must be also. 
	if (remoteAddr == "127.0.0.1") {
		local = "127.0.0.1";
	}

	std::shared_ptr<UdpClientPort> socket = std::make_shared<UdpClientPort>();

	socket->connect(local, 0, remoteAddr, remotePort);

	return createConnection(nodeName, socket);
}

std::shared_ptr<MavLinkConnection>  MavLinkConnectionImpl::connectTcp(const std::string& nodeName, std::string localAddr, const std::string& remoteIpAddr, int remotePort)
{
	std::string local = localAddr;
	// just a little sanity check on the local address, if remoteAddr is localhost then localAddr must be also. 
	if (remoteIpAddr == "127.0.0.1") {
		local = "127.0.0.1";
	}

	std::shared_ptr<TcpClientPort> socket = std::make_shared<TcpClientPort>();

	socket->connect(local, 0, remoteIpAddr, remotePort);

	return createConnection(nodeName, socket);
}

std::shared_ptr<MavLinkConnection>  MavLinkConnectionImpl::connectSerial(const std::string& nodeName, std::string name, int baudRate, const std::string initString)
{
	std::shared_ptr<SerialPort> serial = std::make_shared<SerialPort>();

	int hr = serial->connect(name.c_str(), baudRate);
	if (hr < 0)
		throw std::runtime_error(Utils::stringf("Could not open the serial port %s, error=%d", name.c_str(), hr));

	// send this right away.
	if (initString.size() > 0) {
		hr = serial->write(reinterpret_cast<const uint8_t*>(initString.c_str()), static_cast<int>(initString.size()));
		if (hr < 0)
			throw std::runtime_error(Utils::stringf("Could not send initial string to the serial port %s, error=%d", name.c_str(), hr));
	}

	return createConnection(nodeName, serial);
}

void MavLinkConnectionImpl::startListening(std::shared_ptr<MavLinkConnection> parent, const std::string& nodeName, std::shared_ptr<Port> connectedPort)
{
	name = nodeName;
	con_ = parent;
	close();
	closed = false;
	port = connectedPort;
	read_thread = std::thread{ &MavLinkConnectionImpl::readPackets, this };
	publish_thread_ = std::thread{ &MavLinkConnectionImpl::publishPackets, this };
}

// log every message that is "sent" using sendMessage.
void MavLinkConnectionImpl::startLoggingSendMessage(std::shared_ptr<MavLinkLog> log)
{
	sendLog_ = log;
}

void MavLinkConnectionImpl::stopLoggingSendMessage()
{
	sendLog_ = nullptr;
}

void MavLinkConnectionImpl::close()
{
	closed = true;
	if (port != nullptr) {
		port->close();
		port = nullptr;
	}

	if (read_thread.joinable()) {
		read_thread.join();
	}
	if (publish_thread_.joinable()) {
        msg_available_.post();
		publish_thread_.join();
	}
	sendLog_ = nullptr;
}

bool MavLinkConnectionImpl::isOpen()
{
    return !closed;
}

int MavLinkConnectionImpl::getTargetComponentId()
{
	return this->other_component_id;
}
int MavLinkConnectionImpl::getTargetSystemId()
{
	return this->other_system_id;
}

uint8_t MavLinkConnectionImpl::getNextSequence()
{
	std::lock_guard<std::mutex> guard(buffer_mutex);
	return next_seq++;
}

void MavLinkConnectionImpl::sendMessage(const MavLinkMessage& msg)
{
	if (!closed) {
		if (sendLog_ != nullptr)
		{
			sendLog_->write(msg);
		}
		{
			const mavlink_message_t& m = reinterpret_cast<const mavlink_message_t&>(msg);
			std::lock_guard<std::mutex> guard(buffer_mutex);
			unsigned len = mavlink_msg_to_send_buffer(message_buf, &m);
			try {
				port->write(message_buf, len);
			}
			catch (std::exception& e) {
				throw std::runtime_error(Utils::stringf("MavLinkConnectionImpl: Error sending message on connection '%s', details: %s", name.c_str(), e.what()));
			}
		}
		{
			std::lock_guard<std::mutex> guard(telemetry_mutex_);
			telemetry_.messagesSent++;
		}
	}
}

void MavLinkConnectionImpl::sendMessage(const MavLinkMessageBase& msg)
{
	mavlink_message_t m;
	m.magic = MAVLINK_STX;
	m.msgid = msg.msgid;
	m.sysid = msg.sysid;
	m.compid = msg.compid;
	m.seq = getNextSequence();

	// pack the payload buffer.
	int len = msg.pack(reinterpret_cast<char*>(m.payload64));

	// calculate checksum
	uint8_t crc_extra = mavlink_message_crcs[m.msgid];
	int msglen = mavlink_message_lengths[m.msgid];
	if (m.msgid == MavLinkTelemetry::kMessageId) {
		msglen = 24; // mavlink doesn't know about our custom telemetry message.
	}		
	if (len != msglen) {
		throw std::runtime_error(Utils::stringf("Message length %d doesn't match expected length%d\n", len, msglen));
	}
	m.len = msglen;
	m.checksum = crc_calculate((reinterpret_cast<const uint8_t*>(&m)) + 3, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&m.checksum, reinterpret_cast<const char*>(&m.payload64[0]), msglen);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(crc_extra, &m.checksum);
#endif

	// these macros use old style cast.
	STRICT_MODE_OFF
	mavlink_ck_a(&m) = (uint8_t)(m.checksum & 0xFF);
	mavlink_ck_b(&m) = (uint8_t)(m.checksum >> 8);
	STRICT_MODE_ON

	// send the message, now that it is ready
	sendMessage(reinterpret_cast<const MavLinkMessage&>(m));
}

int MavLinkConnectionImpl::subscribe(MessageHandler handler)
{
	MessageHandlerEntry entry = { static_cast<int>(listeners.size() + 1), handler = handler };
	std::lock_guard<std::mutex> guard(listener_mutex);
	listeners.push_back(entry);
	snapshot_stale = true;
	return entry.id;
}
void MavLinkConnectionImpl::unsubscribe(int id)
{
	std::lock_guard<std::mutex> guard(listener_mutex);
	for (auto ptr = listeners.begin(), end = listeners.end(); ptr != end; ptr++)
	{
		if ((*ptr).id == id)
		{
			listeners.erase(ptr);
			snapshot_stale = true;
			break;
		}
	}
}

void MavLinkConnectionImpl::joinLeftSubscriber(std::shared_ptr<MavLinkConnection> remote, std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg)
{
    // forward messages from our connected node to the remote proxy.
    remote->sendMessage(msg);
}

void MavLinkConnectionImpl::joinRightSubscriber(std::shared_ptr<MavLinkConnection>con, const MavLinkMessage& msg)
{
    // forward messages from remote proxy to local connected node
    this->sendMessage(msg);
}

void MavLinkConnectionImpl::join(std::shared_ptr<MavLinkConnection> remote, bool subscribeToLeft, bool subscribeToRight)
{
    if (subscribeToLeft)
	    this->subscribe(std::bind(&MavLinkConnectionImpl::joinLeftSubscriber, this, remote, std::placeholders::_1, std::placeholders::_2));

    if (subscribeToRight)
	    remote->subscribe(std::bind(&MavLinkConnectionImpl::joinRightSubscriber, this, std::placeholders::_1, std::placeholders::_2));
}

void MavLinkConnectionImpl::readPackets()
{
	CurrentThread::setMaximumPriority();
	std::shared_ptr<Port> safePort = this->port;
	mavlink_message_t msg;
	mavlink_status_t status;
	mavlink_status_t statusBuffer; // intermediate state
	mavlink_message_t msgBuffer; // intermediate state.
	const int MAXBUFFER = 512;
	uint8_t* buffer = new uint8_t[MAXBUFFER];
	statusBuffer.parse_state = MAVLINK_PARSE_STATE_IDLE;
	int channel = 0;
	int hr = 0;
	while (hr == 0 && con_ != nullptr && !closed)
	{
		int read = 0;
		if (safePort->isClosed())
		{
			// hmmm, wait till it is opened?
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		int count = safePort->read(buffer, MAXBUFFER);
		if (count <= 0) {
			// error, so just try again...
			std::lock_guard<std::mutex> guard(telemetry_mutex_);
			telemetry_.crcErrors++;
			continue;
		}
		for (int i = 0; i < count; i++)
		{
			uint8_t frame_state = mavlink_frame_char_buffer(&msgBuffer, &statusBuffer, buffer[i], &msg, &status);

			if (frame_state == MAVLINK_FRAMING_INCOMPLETE) {
				continue;
			}
			else if (frame_state == MAVLINK_FRAMING_BAD_CRC) {
				std::lock_guard<std::mutex> guard(telemetry_mutex_);
				telemetry_.crcErrors++;
			}
			else if (frame_state == MAVLINK_FRAMING_OK)
			{
				int msgId = msg.msgid;

				// pick up the sysid/compid of the remote node we are connected to.
				if (other_system_id == -1) {
					other_system_id = msg.sysid;
					other_component_id = msg.compid;
				}

				if (con_ != nullptr && !closed)
				{
					{
						std::lock_guard<std::mutex> guard(telemetry_mutex_);
						telemetry_.messagesReceived++;
					}
					// queue event for publishing.
                    {
                        std::lock_guard<std::mutex> guard(msg_queue_mutex_);
                        MavLinkMessage& message = reinterpret_cast<MavLinkMessage&>(msg);
                        msg_queue_.push(message);
                    }
					if (waiting_for_msg_) {
						msg_available_.post();
					}
				}
			}
			else {
				std::lock_guard<std::mutex> guard(telemetry_mutex_);
				telemetry_.crcErrors++;
			}
		}	

	} //while

	delete[]  buffer;

} //readPackets

void MavLinkConnectionImpl::drainQueue()
{
	MavLinkMessage message;
	bool hasMsg = true;
	while (hasMsg) {
		hasMsg = false;
		{
			std::lock_guard<std::mutex> guard(msg_queue_mutex_);
			if (!msg_queue_.empty()) {
				message = msg_queue_.front();
				msg_queue_.pop();
				hasMsg = true;
			}
		}
		if (!hasMsg)
		{
			return;
		}
		// publish the message from this thread, this is safer than publishing from the readPackets thread
		// as it ensures we don't lose messages if the listener is slow.
		if (snapshot_stale) {
			// this is tricky, the clear has to be done outside the lock because it is destructing the handlers
			// and the handler might try and call unsubscribe, which needs to be able to grab the lock, otherwise
			// we would get a deadlock.
			snapshot.clear();

			std::lock_guard<std::mutex> guard(listener_mutex);
			snapshot = listeners;
			snapshot_stale = false;
		}
		auto end = snapshot.end();

		auto startTime = std::chrono::system_clock::now();
		std::shared_ptr<MavLinkConnection> sharedPtr = std::shared_ptr<MavLinkConnection>(this->con_);
		for (auto ptr = snapshot.begin(); ptr != end; ptr++)
		{
			try {
				(*ptr).handler(sharedPtr, message);
			}
			catch (std::exception e) {
				Utils::logError("MavLinkConnection subscriber threw exception: %s", e.what());
			}
		}

		{
			auto endTime = std::chrono::system_clock::now();
			auto diff = endTime - startTime;
			long microseconds = static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(diff).count());
			std::lock_guard<std::mutex> guard(telemetry_mutex_);
			telemetry_.messagesHandled++;
			telemetry_.handlerMicroseconds += microseconds;
		}
	}
}

void MavLinkConnectionImpl::publishPackets()
{
	CurrentThread::setMaximumPriority();
	while (!closed) {

		drainQueue();
		
		waiting_for_msg_ = true;
		msg_available_.wait();
		waiting_for_msg_ = false;
	}
}

void MavLinkConnectionImpl::getTelemetry(MavLinkTelemetry& result)
{
	std::lock_guard<std::mutex> guard(telemetry_mutex_);
	result = telemetry_;
	// reset counters 
	telemetry_.crcErrors = 0;
	telemetry_.handlerMicroseconds = 0;
	telemetry_.messagesHandled = 0;
	telemetry_.messagesReceived = 0;
	telemetry_.messagesSent = 0;
	telemetry_.renderTime = 0;
    if (telemetry_.wifiInterfaceName != nullptr) {
        telemetry_.wifiRssi = port->getRssi(telemetry_.wifiInterfaceName);
    }
}
