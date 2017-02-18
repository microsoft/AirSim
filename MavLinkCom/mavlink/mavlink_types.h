#ifndef MAVLINK_TYPES_H_
#define MAVLINK_TYPES_H_

// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
#if (defined _MSC_VER) && (_MSC_VER < 1600)
#error "The C-MAVLink implementation requires Visual Studio 2010 or greater"
#endif

#include <stdint.h>

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

#define MAVLINK_CORE_HEADER_LEN 5 ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and checksum
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)

#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) ///< Maximum packet length

#define MAVLINK_MSG_ID_EXTENDED_MESSAGE 255
#define MAVLINK_EXTENDED_HEADER_LEN 14

#if (defined _MSC_VER) || ((defined __APPLE__) && (defined __MACH__)) || (defined __linux__)
  /* full fledged 32bit++ OS */
  #define MAVLINK_MAX_EXTENDED_PACKET_LEN 65507
#else
  /* small microcontrollers */
  #define MAVLINK_MAX_EXTENDED_PACKET_LEN 2048
#endif

#define MAVLINK_MAX_EXTENDED_PAYLOAD_LEN (MAVLINK_MAX_EXTENDED_PACKET_LEN - MAVLINK_EXTENDED_HEADER_LEN - MAVLINK_NUM_NON_PAYLOAD_BYTES)


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
 * as bitfield ordering isn’t consistent between platforms. The above is intended to be for gcc on x86,
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
	uint16_t checksum; ///< sent at end of packet
	uint8_t magic;   ///< protocol magic marker
	uint8_t len;     ///< Length of payload
	uint8_t seq;     ///< Sequence of packet
	uint8_t sysid;   ///< ID of message sender system/aircraft
	uint8_t compid;  ///< ID of the message sender component
	uint8_t msgid;   ///< ID of message in payload
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
}) mavlink_message_t;

MAVPACKED(
typedef struct __mavlink_extended_message {
       mavlink_message_t base_msg;
       int32_t extended_payload_len;   ///< Length of extended payload if any
       uint8_t extended_payload[MAVLINK_MAX_EXTENDED_PAYLOAD_LEN];
}) mavlink_extended_message_t;

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
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC1
} mavlink_parse_state_t; ///< The state machine for the comm parser

typedef enum {
    MAVLINK_FRAMING_INCOMPLETE=0,
    MAVLINK_FRAMING_OK=1,
    MAVLINK_FRAMING_BAD_CRC=2
} mavlink_framing_t;

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
} mavlink_status_t;

#define MAVLINK_BIG_ENDIAN 0
#define MAVLINK_LITTLE_ENDIAN 1

#endif /* MAVLINK_TYPES_H_ */
