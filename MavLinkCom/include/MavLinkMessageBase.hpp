// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_MavLinkMessageBase_hpp
#define MavLinkCom_MavLinkMessageBase_hpp

#include <stdint.h>
#include <string>
#include <memory>
namespace mavlinkcom_impl {
	class MavLinkConnectionImpl;
	class MavLinkNodeImpl;
}
namespace mavlinkcom
{
	class MavLinkConnection;

	// This is the raw undecoded message, use the msgid field to figure out which strongly typed
	// MavLinkMessageBase subclass can be used to decode this message.  For example, a heartbeat:
	// MavLinkMessage msg;
	// if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
	//     MavLinkHeartbeat heartbeat;
	//     heartbeat.decode(msg);
	//     ...
	// }
	class MavLinkMessage {
	public:
		// These are the raw packet message fields.
		// See MavLinkMessage subclasses for the nice unpacked strongly typed fields.
		uint16_t checksum; ///< sent at end of packet
		uint8_t magic;   ///< protocol magic marker
		uint8_t len;     ///< Length of payload
		uint8_t seq;     ///< Sequence of packet
		uint8_t sysid;   ///< ID of message sender system/aircraft
		uint8_t compid;  ///< ID of the message sender component
		uint8_t msgid;   ///< ID of message in payload
		uint64_t payload64[(255 + 2 + 7) / 8];
	};

	// This is the base class for all the strongly typed messages define in MavLinkMessages.hpp
	class MavLinkMessageBase
	{
	public:
		uint8_t msgid = 0;
		uint8_t sysid = 0;   ///< ID of message sender system/aircraft
		uint8_t compid = 0;  ///< ID of the message sender component

		// unpack the given message
		void decode(const MavLinkMessage& msg);
	protected:
		virtual int pack(char* buffer) const = 0;
		virtual int unpack(const char* buffer) = 0;

		void pack_uint8_t(char* buffer, const uint8_t* field, int offset) const;
		void pack_int8_t(char* buffer, const int8_t* field, int offset) const;
		void pack_int16_t(char* buffer, const int16_t* field, int offset) const;
		void pack_uint16_t(char* buffer, const uint16_t* field, int offset) const;
		void pack_uint32_t(char* buffer, const uint32_t* field, int offset) const;
		void pack_int32_t(char* buffer, const int32_t* field, int offset) const;
		void pack_uint64_t(char* buffer, const uint64_t* field, int offset) const;
		void pack_int64_t(char* buffer, const int64_t* field, int offset) const;
		void pack_float(char* buffer, const float* field, int offset) const;
		void pack_char_array(int len, char* buffer, const char* field, int offset) const;
		void pack_uint8_t_array(int len, char* buffer, const uint8_t* field, int offset) const;
		void pack_int8_t_array(int len, char* buffer, const int8_t* field, int offset) const;
		void pack_uint16_t_array(int len, char* buffer, const uint16_t* field, int offset) const;
		void pack_int16_t_array(int len, char* buffer, const int16_t* field, int offset) const;
		void pack_float_array(int len, char* buffer, const float* field, int offset) const;
		
		void unpack_uint8_t(const char* buffer, uint8_t* field, int offset);
		void unpack_int8_t(const char* buffer, int8_t* field, int offset);
		void unpack_int16_t(const char* buffer, int16_t* field, int offset);
		void unpack_uint16_t(const char* buffer, uint16_t* field, int offset);
		void unpack_uint32_t(const char* buffer, uint32_t* field, int offset);
		void unpack_int32_t(const char* buffer, int32_t* field, int offset);
		void unpack_uint64_t(const char* buffer, uint64_t* field, int offset);
		void unpack_int64_t(const char* buffer, int64_t* field, int offset);
		void unpack_float(const char* buffer, float* field, int offset);
		void unpack_char_array(int len, const char* buffer, char* field, int offset);
		void unpack_uint8_t_array(int len, const char* buffer, uint8_t* field, int offset);
		void unpack_int8_t_array(int len, const char* buffer, int8_t* field, int offset);
		void unpack_uint16_t_array(int len, const char* buffer, uint16_t* field, int offset);
		void unpack_int16_t_array(int len, const char* buffer, int16_t* field, int offset);
		void unpack_float_array(int len, const char* buffer, float* field, int offset);

		friend class mavlinkcom_impl::MavLinkConnectionImpl;
	};

	// Base class for all strongly typed MavLinkCommand classes defined in MavLinkMessages.hpp
	class MavLinkCommand {
	public:
		uint16_t command = 0;
	protected:
		virtual void pack() = 0;
		virtual void unpack() = 0;

		float param1 = 0;
		float param2 = 0;
		float param3 = 0;
		float param4 = 0;
		float param5 = 0;
		float param6 = 0;
		float param7 = 0;

		friend class mavlinkcom_impl::MavLinkNodeImpl;
	};


	// The location of a landing area captured from a downward facing camera
	class MavLinkTelemetry : public MavLinkMessageBase {
	public:
		const static uint8_t kMessageId = 204; // in the user range 180-229.
		MavLinkTelemetry() { msgid = kMessageId; }
		long messagesSent;		 // number of messages sent since the last telemetry message
		long messagesReceived;	 // number of messages received since the last telemetry message
		long messagesHandled;	 // number of messages handled since the last telemetry message
		long crcErrors;			 // # crc errors detected in mavlink stream since the last telemetry message
		long handlerMicroseconds; // total time spent in the handlers in microseconds since the last telemetry message
		long renderTime;         // total time spent rendering frames since the last telemetry message
	protected:
		virtual int pack(char* buffer) const;
		virtual int unpack(const char* buffer);
	};

}

#endif
