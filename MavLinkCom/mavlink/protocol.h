#pragma once

#include "string.h"
#include "mavlink_types.h"

/* 
   If you want MAVLink on a system that is native big-endian,
   you need to define NATIVE_BIG_ENDIAN
*/
#ifdef NATIVE_BIG_ENDIAN
# define MAVLINK_NEED_BYTE_SWAP (MAVLINK_ENDIAN == MAVLINK_LITTLE_ENDIAN)
#else
# define MAVLINK_NEED_BYTE_SWAP (MAVLINK_ENDIAN != MAVLINK_LITTLE_ENDIAN)
#endif

#ifndef MAVLINK_STACK_BUFFER
#define MAVLINK_STACK_BUFFER 0
#endif

#ifndef MAVLINK_AVOID_GCC_STACK_BUG
# define MAVLINK_AVOID_GCC_STACK_BUG defined(__GNUC__)
#endif

#ifndef MAVLINK_ASSERT
#define MAVLINK_ASSERT(x)
#endif

#ifndef MAVLINK_START_UART_SEND
#define MAVLINK_START_UART_SEND(chan, length)
#endif

#ifndef MAVLINK_END_UART_SEND
#define MAVLINK_END_UART_SEND(chan, length)
#endif

/* option to provide alternative implementation of mavlink_helpers.h */
#ifdef MAVLINK_SEPARATE_HELPERS

    #define MAVLINK_HELPER

    /* decls in sync with those in mavlink_helpers.h */
    #ifndef MAVLINK_GET_CHANNEL_STATUS
    MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
    #endif
    MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan);
    MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                          uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra);
    MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
                                                     uint8_t min_length, uint8_t length, uint8_t crc_extra);
    #ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
    MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid, const char *packet,
                                                        uint8_t min_length, uint8_t length, uint8_t crc_extra);
    #endif
    MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg);
    MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg);
    MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c);
    MAVLINK_HELPER uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, 
						     mavlink_status_t* status,
						     uint8_t c, 
						     mavlink_message_t* r_message, 
						     mavlink_status_t* r_mavlink_status);
    MAVLINK_HELPER uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
    MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
    MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index,
                               uint8_t* r_bit_index, uint8_t* buffer);
    MAVLINK_HELPER const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid);
    #ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
    MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);
    MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg);
    #endif

#else

    #define MAVLINK_HELPER static inline
    #include "mavlink_helpers.h"

#endif // MAVLINK_SEPARATE_HELPERS


/**
 * @brief Get the required buffer size for this message
 */
static inline uint16_t mavlink_msg_get_send_buffer_length(const mavlink_message_t* msg)
{
	if (msg->magic == MAVLINK_STX_MAVLINK1) {
		return msg->len + MAVLINK_CORE_HEADER_MAVLINK1_LEN+1 + 2;
	}
    	uint16_t signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED)?MAVLINK_SIGNATURE_BLOCK_LEN:0;
	return msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES + signature_len;
}

#if MAVLINK_NEED_BYTE_SWAP
static inline void byte_swap_2(char *dst, const char *src)
{
	dst[0] = src[1];
	dst[1] = src[0];
}
static inline void byte_swap_4(char *dst, const char *src)
{
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}
static inline void byte_swap_8(char *dst, const char *src)
{
	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}
#elif !MAVLINK_ALIGNED_FIELDS
static inline void byte_copy_2(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
}
static inline void byte_copy_4(char *dst, const char *src)
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}
static inline void byte_copy_8(char *dst, const char *src)
{
	memcpy(dst, src, 8);
}
#endif

#define _mav_put_uint8_t(buf, wire_offset, b) buf[wire_offset] = (uint8_t)b
#define _mav_put_int8_t(buf, wire_offset, b)  buf[wire_offset] = (int8_t)b
#define _mav_put_char(buf, wire_offset, b)    buf[wire_offset] = b

#if MAVLINK_NEED_BYTE_SWAP
#define _mav_put_uint16_t(buf, wire_offset, b) byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_int16_t(buf, wire_offset, b)  byte_swap_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint32_t(buf, wire_offset, b) byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_int32_t(buf, wire_offset, b)  byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint64_t(buf, wire_offset, b) byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_int64_t(buf, wire_offset, b)  byte_swap_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_float(buf, wire_offset, b)    byte_swap_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_double(buf, wire_offset, b)   byte_swap_8(&buf[wire_offset], (const char *)&b)
#elif !MAVLINK_ALIGNED_FIELDS
#define _mav_put_uint16_t(buf, wire_offset, b) byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_int16_t(buf, wire_offset, b)  byte_copy_2(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint32_t(buf, wire_offset, b) byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_int32_t(buf, wire_offset, b)  byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_uint64_t(buf, wire_offset, b) byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_int64_t(buf, wire_offset, b)  byte_copy_8(&buf[wire_offset], (const char *)&b)
#define _mav_put_float(buf, wire_offset, b)    byte_copy_4(&buf[wire_offset], (const char *)&b)
#define _mav_put_double(buf, wire_offset, b)   byte_copy_8(&buf[wire_offset], (const char *)&b)
#else
#define _mav_put_uint16_t(buf, wire_offset, b) *(uint16_t *)&buf[wire_offset] = b
#define _mav_put_int16_t(buf, wire_offset, b)  *(int16_t *)&buf[wire_offset] = b
#define _mav_put_uint32_t(buf, wire_offset, b) *(uint32_t *)&buf[wire_offset] = b
#define _mav_put_int32_t(buf, wire_offset, b)  *(int32_t *)&buf[wire_offset] = b
#define _mav_put_uint64_t(buf, wire_offset, b) *(uint64_t *)&buf[wire_offset] = b
#define _mav_put_int64_t(buf, wire_offset, b)  *(int64_t *)&buf[wire_offset] = b
#define _mav_put_float(buf, wire_offset, b)    *(float *)&buf[wire_offset] = b
#define _mav_put_double(buf, wire_offset, b)   *(double *)&buf[wire_offset] = b
#endif

/*
  like memcpy(), but if src is NULL, do a memset to zero
*/
static inline void mav_array_memcpy(void *dest, const void *src, size_t n)
{
	if (src == NULL) {
		memset(dest, 0, n);
	} else {
		memcpy(dest, src, n);
	}
}

/*
 * Place a char array into a buffer
 */
static inline void _mav_put_char_array(char *buf, uint8_t wire_offset, const char *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a uint8_t array into a buffer
 */
static inline void _mav_put_uint8_t_array(char *buf, uint8_t wire_offset, const uint8_t *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}

/*
 * Place a int8_t array into a buffer
 */
static inline void _mav_put_int8_t_array(char *buf, uint8_t wire_offset, const int8_t *b, uint8_t array_length)
{
	mav_array_memcpy(&buf[wire_offset], b, array_length);

}

#if MAVLINK_NEED_BYTE_SWAP
#define _MAV_PUT_ARRAY(TYPE, V) \
static inline void _mav_put_ ## TYPE ##_array(char *buf, uint8_t wire_offset, const TYPE *b, uint8_t array_length) \
{ \
	if (b == NULL) { \
		memset(&buf[wire_offset], 0, array_length*sizeof(TYPE)); \
	} else { \
		uint16_t i; \
		for (i=0; i<array_length; i++) { \
			_mav_put_## TYPE (buf, wire_offset+(i*sizeof(TYPE)), b[i]); \
		} \
	} \
}
#else
#define _MAV_PUT_ARRAY(TYPE, V)					\
static inline void _mav_put_ ## TYPE ##_array(char *buf, uint8_t wire_offset, const TYPE *b, uint8_t array_length) \
{ \
	mav_array_memcpy(&buf[wire_offset], b, array_length*sizeof(TYPE)); \
}
#endif

_MAV_PUT_ARRAY(uint16_t, u16)
_MAV_PUT_ARRAY(uint32_t, u32)
_MAV_PUT_ARRAY(uint64_t, u64)
_MAV_PUT_ARRAY(int16_t,  i16)
_MAV_PUT_ARRAY(int32_t,  i32)
_MAV_PUT_ARRAY(int64_t,  i64)
_MAV_PUT_ARRAY(float,    f)
_MAV_PUT_ARRAY(double,   d)

#define _MAV_RETURN_char(msg, wire_offset)             (char)_MAV_PAYLOAD(msg)[wire_offset]
#define _MAV_RETURN_int8_t(msg, wire_offset)   (int8_t)_MAV_PAYLOAD(msg)[wire_offset]
#define _MAV_RETURN_uint8_t(msg, wire_offset) (uint8_t)_MAV_PAYLOAD(msg)[wire_offset]

#if MAVLINK_NEED_BYTE_SWAP
#define _MAV_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_swap_## SIZE((char*)&r, &_MAV_PAYLOAD(msg)[ofs]); return r; }

_MAV_MSG_RETURN_TYPE(uint16_t, 2)
_MAV_MSG_RETURN_TYPE(int16_t,  2)
_MAV_MSG_RETURN_TYPE(uint32_t, 4)
_MAV_MSG_RETURN_TYPE(int32_t,  4)
_MAV_MSG_RETURN_TYPE(uint64_t, 8)
_MAV_MSG_RETURN_TYPE(int64_t,  8)
_MAV_MSG_RETURN_TYPE(float,    4)
_MAV_MSG_RETURN_TYPE(double,   8)

#elif !MAVLINK_ALIGNED_FIELDS
#define _MAV_MSG_RETURN_TYPE(TYPE, SIZE) \
static inline TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ TYPE r; byte_copy_## SIZE((char*)&r, &_MAV_PAYLOAD(msg)[ofs]); return r; }

_MAV_MSG_RETURN_TYPE(uint16_t, 2)
_MAV_MSG_RETURN_TYPE(int16_t,  2)
_MAV_MSG_RETURN_TYPE(uint32_t, 4)
_MAV_MSG_RETURN_TYPE(int32_t,  4)
_MAV_MSG_RETURN_TYPE(uint64_t, 8)
_MAV_MSG_RETURN_TYPE(int64_t,  8)
_MAV_MSG_RETURN_TYPE(float,    4)
_MAV_MSG_RETURN_TYPE(double,   8)
#else // nicely aligned, no swap
#define _MAV_MSG_RETURN_TYPE(TYPE) \
static inline TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ return *(const TYPE *)(&_MAV_PAYLOAD(msg)[ofs]);}

_MAV_MSG_RETURN_TYPE(uint16_t)
_MAV_MSG_RETURN_TYPE(int16_t)
_MAV_MSG_RETURN_TYPE(uint32_t)
_MAV_MSG_RETURN_TYPE(int32_t)
_MAV_MSG_RETURN_TYPE(uint64_t)
_MAV_MSG_RETURN_TYPE(int64_t)
_MAV_MSG_RETURN_TYPE(float)
_MAV_MSG_RETURN_TYPE(double)
#endif // MAVLINK_NEED_BYTE_SWAP

static inline uint16_t _MAV_RETURN_char_array(const mavlink_message_t *msg, char *value, 
						     uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MAV_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _MAV_RETURN_uint8_t_array(const mavlink_message_t *msg, uint8_t *value, 
							uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MAV_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

static inline uint16_t _MAV_RETURN_int8_t_array(const mavlink_message_t *msg, int8_t *value, 
						       uint8_t array_length, uint8_t wire_offset)
{
	memcpy(value, &_MAV_PAYLOAD(msg)[wire_offset], array_length);
	return array_length;
}

#if MAVLINK_NEED_BYTE_SWAP
#define _MAV_RETURN_ARRAY(TYPE, V) \
static inline uint16_t _MAV_RETURN_## TYPE ##_array(const mavlink_message_t *msg, TYPE *value, \
							 uint8_t array_length, uint8_t wire_offset) \
{ \
	uint16_t i; \
	for (i=0; i<array_length; i++) { \
		value[i] = _MAV_RETURN_## TYPE (msg, wire_offset+(i*sizeof(value[0]))); \
	} \
	return array_length*sizeof(value[0]); \
}
#else
#define _MAV_RETURN_ARRAY(TYPE, V)					\
static inline uint16_t _MAV_RETURN_## TYPE ##_array(const mavlink_message_t *msg, TYPE *value, \
							 uint8_t array_length, uint8_t wire_offset) \
{ \
	memcpy(value, &_MAV_PAYLOAD(msg)[wire_offset], array_length*sizeof(TYPE)); \
	return array_length*sizeof(TYPE); \
}
#endif

_MAV_RETURN_ARRAY(uint16_t, u16)
_MAV_RETURN_ARRAY(uint32_t, u32)
_MAV_RETURN_ARRAY(uint64_t, u64)
_MAV_RETURN_ARRAY(int16_t,  i16)
_MAV_RETURN_ARRAY(int32_t,  i32)
_MAV_RETURN_ARRAY(int64_t,  i64)
_MAV_RETURN_ARRAY(float,    f)
_MAV_RETURN_ARRAY(double,   d)


