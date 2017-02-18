// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkMessageBase.hpp"
#include "MavLinkConnection.hpp"
#include "Utils.hpp"
using namespace common_utils;

STRICT_MODE_OFF
#define MAVLINK_PACKED
#include "../mavlink/common/mavlink.h"
#include "../mavlink/mavlink_types.h"
#include "../mavlink/mavlink_helpers.h"
STRICT_MODE_ON


using namespace mavlinkcom;
void MavLinkMessageBase::decode(const MavLinkMessage& msg) {
    // unpack the message...
    this->msgid = msg.msgid;
    unpack(reinterpret_cast<const char*>(msg.payload64));

}


void MavLinkMessageBase::pack_uint8_t(char* buffer, const uint8_t* field, int offset) const {
    buffer[offset] = *reinterpret_cast<const char*>(field);
}
void MavLinkMessageBase::pack_int8_t(char* buffer, const int8_t* field, int offset) const {
    buffer[offset] = *reinterpret_cast<const char*>(field);
}
void MavLinkMessageBase::pack_int16_t(char* buffer, const int16_t* field, int offset) const {
    _mav_put_int16_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_uint16_t(char* buffer, const uint16_t* field, int offset) const {
    _mav_put_uint16_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_uint32_t(char* buffer, const uint32_t* field, int offset) const {
    _mav_put_uint32_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_int32_t(char* buffer, const int32_t* field, int offset) const {
    _mav_put_int32_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_uint64_t(char* buffer, const uint64_t* field, int offset) const {
    _mav_put_uint64_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_int64_t(char* buffer, const int64_t* field, int offset) const {
    _mav_put_int64_t(buffer, offset, *field);
}
void MavLinkMessageBase::pack_float(char* buffer, const float* field, int offset) const {
    _mav_put_float(buffer, offset, *field);
}
void MavLinkMessageBase::pack_char_array(int len, char* buffer, const char* field, int offset) const {
    _mav_put_char_array(buffer, offset, field, len);
}
void MavLinkMessageBase::pack_uint8_t_array(int len, char* buffer, const uint8_t* field, int offset) const {
    _mav_put_uint8_t_array(buffer, offset, field, len);
}
void MavLinkMessageBase::pack_int8_t_array(int len, char* buffer, const int8_t* field, int offset) const {
    _mav_put_int8_t_array(buffer, offset, field, len);
}
void MavLinkMessageBase::pack_uint16_t_array(int len, char* buffer, const uint16_t* field, int offset) const {
    _mav_put_uint16_t_array(buffer, offset, field, len);
}
void MavLinkMessageBase::pack_int16_t_array(int len, char* buffer, const int16_t* field, int offset) const {
    _mav_put_int16_t_array(buffer, offset, field, len);
}
void MavLinkMessageBase::pack_float_array(int len, char* buffer, const float* field, int offset) const {
    _mav_put_float_array(buffer, offset, field, len);
}


#if MAVLINK_NEED_BYTE_SWAP
#define _mav_get_uint16_t(buf, wire_offset, b) byte_swap_2(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int16_t(buf, wire_offset, b)  byte_swap_2(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_uint32_t(buf, wire_offset, b) byte_swap_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int32_t(buf, wire_offset, b)  byte_swap_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_uint64_t(buf, wire_offset, b) byte_swap_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int64_t(buf, wire_offset, b)  byte_swap_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_float(buf, wire_offset, b)    byte_swap_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_double(buf, wire_offset, b)   byte_swap_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#elif !MAVLINK_ALIGNED_FIELDS
#define _mav_get_uint16_t(buf, wire_offset, b) byte_copy_2(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int16_t(buf, wire_offset, b)  byte_copy_2(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_uint32_t(buf, wire_offset, b) byte_copy_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int32_t(buf, wire_offset, b)  byte_copy_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_uint64_t(buf, wire_offset, b) byte_copy_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_int64_t(buf, wire_offset, b)  byte_copy_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_float(buf, wire_offset, b)    byte_copy_4(reinterpret_cast<char *>(b),&buf[wire_offset], )
#define _mav_get_double(buf, wire_offset, b)   byte_copy_8(reinterpret_cast<char *>(b),&buf[wire_offset], )
#else
#define _mav_get_uint16_t(buf, wire_offset, b) *b = *reinterpret_cast<const uint16_t *>(&buf[wire_offset])
#define _mav_get_int16_t(buf, wire_offset, b)  *b = *reinterpret_cast<const int16_t * >(&buf[wire_offset])
#define _mav_get_uint32_t(buf, wire_offset, b) *b = *reinterpret_cast<const uint32_t *>(&buf[wire_offset])
#define _mav_get_int32_t(buf, wire_offset, b)  *b = *reinterpret_cast<const int32_t * >(&buf[wire_offset])
#define _mav_get_uint64_t(buf, wire_offset, b) *b = *reinterpret_cast<const uint64_t *>(&buf[wire_offset])
#define _mav_get_int64_t(buf, wire_offset, b)  *b = *reinterpret_cast<const int64_t * >(&buf[wire_offset])
#define _mav_get_float(buf, wire_offset, b)    *b = *reinterpret_cast<const float *   >(&buf[wire_offset])
#define _mav_get_double(buf, wire_offset, b)   *b = *reinterpret_cast<const double *  >(&buf[wire_offset])
#endif


void MavLinkMessageBase::unpack_uint8_t(const char* buffer, uint8_t* field, int offset) {
    *reinterpret_cast<char*>(field) = buffer[offset];
}
void MavLinkMessageBase::unpack_int8_t(const char* buffer, int8_t* field, int offset) {
    *reinterpret_cast<char*>(field) = buffer[offset];
}
void MavLinkMessageBase::unpack_int16_t(const char* buffer, int16_t* field, int offset) {
    _mav_get_int16_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint16_t(const char* buffer, uint16_t* field, int offset) {
    _mav_get_uint16_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint32_t(const char* buffer, uint32_t* field, int offset) {
    _mav_get_uint32_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_int32_t(const char* buffer, int32_t* field, int offset) {
    _mav_get_int32_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint64_t(const char* buffer, uint64_t* field, int offset) {
    _mav_get_uint64_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_int64_t(const char* buffer, int64_t* field, int offset) {
    _mav_get_int64_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_float(const char* buffer, float* field, int offset) {
    _mav_get_float(buffer, offset, field);
}
void MavLinkMessageBase::unpack_char_array(int len, const char* buffer, char* field, int offset) {
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_uint8_t_array(int len, const char* buffer, uint8_t* field, int offset) {
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_int8_t_array(int len, const char* buffer, int8_t* field, int offset) {
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_uint16_t_array(int len, const char* buffer, uint16_t* field, int offset) {
    for (int i = 0; i < len; i++) {
        _mav_get_uint16_t(buffer, offset, field);
        offset += sizeof(uint16_t);
        field++;
    }
}
void MavLinkMessageBase::unpack_int16_t_array(int len, const char* buffer, int16_t* field, int offset) {
    for (int i = 0; i < len; i++) {
        _mav_get_int16_t(buffer, offset, field);
        offset += sizeof(int16_t);
        field++;
    }
}
void MavLinkMessageBase::unpack_float_array(int len, const char* buffer, float* field, int offset) {
    for (int i = 0; i < len; i++) {
        _mav_get_float(buffer, offset, field);
        offset += sizeof(float);
        field++;
    }
}
