#pragma once

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

// Macro to define packed structures
#ifdef __GNUC__
  #define MAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#else
  #define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef MAVLINK_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#endif

#define MAVLINK_CORE_HEADER_LEN 9 ///< Length of core header (of the comm. layer)
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5 ///< Length of MAVLink1 core header (of the comm. layer)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)

#define MAVLINK_SIGNATURE_BLOCK_LEN 13

#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_SIGNATURE_BLOCK_LEN) ///< Maximum packet length

/**
 * Old-style 4 byte param union
 *
 * This struct is the data format to be used when sending
 * parameters. The parameter should be copied to the native
 * type (without type conversion)
 * and re-instanted on the receiving side using the
 * native type as well.
 */
MAVPACKED(
typedef struct param_union {
	union {
		float param_float;
		int32_t param_int32;
		uint32_t param_uint32;
		int16_t param_int16;
		uint16_t param_uint16;
		int8_t param_int8;
		uint8_t param_uint8;
		uint8_t bytes[4];
	};
	uint8_t type;
}) mavlink_param_union_t;


/**
 * New-style 8 byte param union
 * mavlink_param_union_double_t will be 8 bytes long, and treated as needing 8 byte alignment for the purposes of MAVLink 1.0 field ordering.
 * The mavlink_param_union_double_t will be treated as a little-endian structure.
 *
 * If is_double is 1 then the type is a double, and the remaining 63 bits are the double, with the lowest bit of the mantissa zero.
 * The intention is that by replacing the is_double bit with 0 the type can be directly used as a double (as the is_double bit corresponds to the
 * lowest mantissa bit of a double). If is_double is 0 then mavlink_type gives the type in the union.
 * The mavlink_types.h header will also need to have shifts/masks to define the bit boundaries in the above,
 * as bitfield ordering isn't consistent between platforms. The above is intended to be for gcc on x86,
 * which should be the same as gcc on little-endian arm. When using shifts/masks the value will be treated as a 64 bit unsigned number,
 * and the bits pulled out using the shifts/masks.
*/
MAVPACKED(
typedef struct param_union_extended {
    union {
    struct {
        uint8_t is_double:1;
        uint8_t mavlink_type:7;
        union {
            char c;
            uint8_t uint8;
            int8_t int8;
            uint16_t uint16;
            int16_t int16;
            uint32_t uint32;
            int32_t int32;
            float f;
            uint8_t align[7];
        };
    };
    uint8_t data[8];
    };
}) mavlink_param_union_double_t;

/**
 * This structure is required to make the mavlink_send_xxx convenience functions
 * work, as it tells the library what the current system and component ID are.
 */
MAVPACKED(
typedef struct __mavlink_system {
    uint8_t sysid;   ///< Used by the MAVLink message_xx_send() convenience function
    uint8_t compid;  ///< Used by the MAVLink message_xx_send() convenience function
}) mavlink_system_t;

MAVPACKED(
typedef struct __mavlink_message {
	uint16_t checksum;      ///< sent at end of packet
	uint8_t magic;          ///< protocol magic marker
	uint8_t len;            ///< Length of payload
	uint8_t incompat_flags; ///< flags that must be understood
	uint8_t compat_flags;   ///< flags that can be ignored if not understood
	uint8_t seq;            ///< Sequence of packet
	uint8_t sysid;          ///< ID of message sender system/aircraft
	uint8_t compid;         ///< ID of the message sender component
	uint32_t msgid:24;      ///< ID of message in payload
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
	uint8_t ck[2];          ///< incoming checksum bytes
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
}) mavlink_message_t;

typedef enum {
	MAVLINK_TYPE_CHAR     = 0,
	MAVLINK_TYPE_UINT8_T  = 1,
	MAVLINK_TYPE_INT8_T   = 2,
	MAVLINK_TYPE_UINT16_T = 3,
	MAVLINK_TYPE_INT16_T  = 4,
	MAVLINK_TYPE_UINT32_T = 5,
	MAVLINK_TYPE_INT32_T  = 6,
	MAVLINK_TYPE_UINT64_T = 7,
	MAVLINK_TYPE_INT64_T  = 8,
	MAVLINK_TYPE_FLOAT    = 9,
	MAVLINK_TYPE_DOUBLE   = 10
} mavlink_message_type_t;

#define MAVLINK_MAX_FIELDS 64

typedef struct __mavlink_field_info {
	const char *name;                 // name of this field
        const char *print_format;         // printing format hint, or NULL
        mavlink_message_type_t type;      // type of this field
        unsigned int array_length;        // if non-zero, field is an array
        unsigned int wire_offset;         // offset of each field in the payload
        unsigned int structure_offset;    // offset in a C structure
} mavlink_field_info_t;

// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
typedef struct __mavlink_message_info {
	uint32_t msgid;                                        // message ID
	const char *name;                                      // name of the message
	unsigned num_fields;                                   // how many fields in this message
	mavlink_field_info_t fields[MAVLINK_MAX_FIELDS];       // field information
} mavlink_message_info_t;

#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes
#define mavlink_ck_a(msg) *((msg)->len + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))
#define mavlink_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))

typedef enum {
    MAVLINK_COMM_0,
    MAVLINK_COMM_1,
    MAVLINK_COMM_2,
    MAVLINK_COMM_3
} mavlink_channel_t;

/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#ifndef MAVLINK_COMM_NUM_BUFFERS
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
# define MAVLINK_COMM_NUM_BUFFERS 16
#else
# define MAVLINK_COMM_NUM_BUFFERS 4
#endif
#endif

typedef enum {
    MAVLINK_PARSE_STATE_UNINIT=0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,
    MAVLINK_PARSE_STATE_GOT_MSGID2,
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC1,
    MAVLINK_PARSE_STATE_SIGNATURE_WAIT
} mavlink_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    MAVLINK_FRAMING_INCOMPLETE=0,
    MAVLINK_FRAMING_OK=1,
    MAVLINK_FRAMING_BAD_CRC=2,
    MAVLINK_FRAMING_BAD_SIGNATURE=3
} mavlink_framing_t;

#define MAVLINK_STATUS_FLAG_IN_MAVLINK1  1 // last incoming packet was MAVLink1
#define MAVLINK_STATUS_FLAG_OUT_MAVLINK1 2 // generate MAVLink1 by default
#define MAVLINK_STATUS_FLAG_IN_SIGNED    4 // last incoming packet was signed and validated
#define MAVLINK_STATUS_FLAG_IN_BADSIG    8 // last incoming packet had a bad signature

#define MAVLINK_STX_MAVLINK1 0xFE          // marker for old protocol

typedef struct __mavlink_status {
    uint8_t msg_received;               ///< Number of received messages
    uint8_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    mavlink_parse_state_t parse_state;  ///< Parsing state machine
    uint8_t packet_idx;                 ///< Index in current packet
    uint8_t current_rx_seq;             ///< Sequence number of last packet received
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t flags;                      ///< MAVLINK_STATUS_FLAG_*
    uint8_t signature_wait;             ///< number of signature bytes left to receive
    struct __mavlink_signing *signing;  ///< optional signing state
    struct __mavlink_signing_streams *signing_streams; ///< global record of stream timestamps
} mavlink_status_t;

/*
  a callback function to allow for accepting unsigned packets
 */
typedef bool (*mavlink_accept_unsigned_t)(const mavlink_status_t *status, uint32_t msgid);

/*
  flags controlling signing
 */
#define MAVLINK_SIGNING_FLAG_SIGN_OUTGOING 1    ///< Enable outgoing signing

/*
  state of MAVLink signing for this channel
 */
typedef struct __mavlink_signing {
    uint8_t flags;                     ///< MAVLINK_SIGNING_FLAG_*
    uint8_t link_id;                   ///< Same as MAVLINK_CHANNEL
    uint64_t timestamp;                ///< Timestamp, in microseconds since UNIX epoch GMT
    uint8_t secret_key[32];
    mavlink_accept_unsigned_t accept_unsigned_callback;
} mavlink_signing_t;

/*
  timestamp state of each logical signing stream. This needs to be the same structure for all
  connections in order to be secure
 */
#ifndef MAVLINK_MAX_SIGNING_STREAMS
#define MAVLINK_MAX_SIGNING_STREAMS 16
#endif
typedef struct __mavlink_signing_streams {
    uint16_t num_signing_streams;
    struct __mavlink_signing_stream {
        uint8_t link_id;              ///< ID of the link (MAVLINK_CHANNEL)
        uint8_t sysid;                ///< Remote system ID
        uint8_t compid;               ///< Remote component ID
        uint8_t timestamp_bytes[6];   ///< Timestamp, in microseconds since UNIX epoch GMT
    } stream[MAVLINK_MAX_SIGNING_STREAMS];
} mavlink_signing_streams_t;


#define MAVLINK_BIG_ENDIAN 0
#define MAVLINK_LITTLE_ENDIAN 1

#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM    1
#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT 2

/*
  entry in table of information about each message type
 */
typedef struct __mavlink_msg_entry {
	uint32_t msgid;
	uint8_t crc_extra;
        uint8_t min_msg_len;       // minimum message length
        uint8_t max_msg_len;       // maximum message length (e.g. including mavlink2 extensions)
        uint8_t flags;             // MAV_MSG_ENTRY_FLAG_*
	uint8_t target_system_ofs; // payload offset to target_system, or 0
	uint8_t target_component_ofs; // payload offset to target_component, or 0
} mavlink_msg_entry_t;

/*
  incompat_flags bits
 */
#define MAVLINK_IFLAG_SIGNED  0x01
#define MAVLINK_IFLAG_MASK    0x01 // mask of all understood bits

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif
