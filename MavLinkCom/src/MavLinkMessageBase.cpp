// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "StrictMode.hpp"

STRICT_MODE_OFF
#define MAVLINK_PACKED
#include "../mavlink/common/mavlink.h"
#include "../mavlink/mavlink_types.h"
#include "../mavlink/mavlink_helpers.h"
STRICT_MODE_ON


#include "MavLinkConnection.hpp"
#include "MavLinkMessageBase.hpp"
#include "Utils.hpp"
#include <sstream>
#include <cmath>

using namespace mavlink_utils;
using namespace mavlinkcom;

void MavLinkMessageBase::decode(const MavLinkMessage& msg)
{
    // unpack the message...
    this->msgid = msg.msgid;
    this->protocol_version = msg.protocol_version;
    unpack(reinterpret_cast<const char*>(msg.payload64));
}

void MavLinkMessageBase::encode(MavLinkMessage& msg) const {

    msg.msgid = this->msgid;
    msg.sysid = this->sysid;
    msg.compid = this->compid;
    msg.protocol_version = this->protocol_version;
    // pack the payload buffer.
    int len = this->pack(reinterpret_cast<char*>(msg.payload64));
    msg.len = len;
}


void MavLinkMessageBase::pack_uint8_t(char* buffer, const uint8_t* field, int offset) const
{
    buffer[offset] = *reinterpret_cast<const char*>(field);
}
void MavLinkMessageBase::pack_int8_t(char* buffer, const int8_t* field, int offset) const
{
    buffer[offset] = *reinterpret_cast<const char*>(field);
}
void MavLinkMessageBase::pack_int16_t(char* buffer, const int16_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_int16_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_uint16_t(char* buffer, const uint16_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_uint16_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_uint32_t(char* buffer, const uint32_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_uint32_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_int32_t(char* buffer, const int32_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_int32_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_uint64_t(char* buffer, const uint64_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_uint64_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_int64_t(char* buffer, const int64_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_int64_t(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_float(char* buffer, const float* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_float(buffer, offset, *field);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_char_array(int len, char* buffer, const char* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_char_array(buffer, offset, field, len);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_uint8_t_array(int len, char* buffer, const uint8_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_uint8_t_array(buffer, offset, field, len);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_int8_t_array(int len, char* buffer, const int8_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_int8_t_array(buffer, offset, field, len);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_uint16_t_array(int len, char* buffer, const uint16_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_uint16_t_array(buffer, offset, field, len);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_int16_t_array(int len, char* buffer, const int16_t* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_int16_t_array(buffer, offset, field, len);
    STRICT_MODE_ON
}
void MavLinkMessageBase::pack_float_array(int len, char* buffer, const float* field, int offset) const
{
    STRICT_MODE_OFF
    _mav_put_float_array(buffer, offset, field, len);
    STRICT_MODE_ON
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


void MavLinkMessageBase::unpack_uint8_t(const char* buffer, uint8_t* field, int offset)
{
    *reinterpret_cast<char*>(field) = buffer[offset];
}
void MavLinkMessageBase::unpack_int8_t(const char* buffer, int8_t* field, int offset)
{
    *reinterpret_cast<char*>(field) = buffer[offset];
}
void MavLinkMessageBase::unpack_int16_t(const char* buffer, int16_t* field, int offset)
{
    _mav_get_int16_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint16_t(const char* buffer, uint16_t* field, int offset)
{
    _mav_get_uint16_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint32_t(const char* buffer, uint32_t* field, int offset)
{
    _mav_get_uint32_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_int32_t(const char* buffer, int32_t* field, int offset)
{
    _mav_get_int32_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_uint64_t(const char* buffer, uint64_t* field, int offset)
{
    _mav_get_uint64_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_int64_t(const char* buffer, int64_t* field, int offset)
{
    _mav_get_int64_t(buffer, offset, field);
}
void MavLinkMessageBase::unpack_float(const char* buffer, float* field, int offset)
{
    _mav_get_float(buffer, offset, field);
}
void MavLinkMessageBase::unpack_char_array(int len, const char* buffer, char* field, int offset)
{
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_uint8_t_array(int len, const char* buffer, uint8_t* field, int offset)
{
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_int8_t_array(int len, const char* buffer, int8_t* field, int offset)
{
    memcpy(field, &buffer[offset], len);
}
void MavLinkMessageBase::unpack_uint16_t_array(int len, const char* buffer, uint16_t* field, int offset)
{
    for (int i = 0; i < len; i++)
    {
        _mav_get_uint16_t(buffer, offset, field);
        offset += sizeof(uint16_t);
        field++;
    }
}
void MavLinkMessageBase::unpack_int16_t_array(int len, const char* buffer, int16_t* field, int offset)
{
    for (int i = 0; i < len; i++)
    {
        _mav_get_int16_t(buffer, offset, field);
        offset += sizeof(int16_t);
        field++;
    }
}
void MavLinkMessageBase::unpack_float_array(int len, const char* buffer, float* field, int offset)
{
    for (int i = 0; i < len; i++)
    {
        _mav_get_float(buffer, offset, field);
        offset += sizeof(float);
        field++;
    }
}

int MavLinkTelemetry::pack(char* buffer) const {
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->messagesSent), 0);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->messagesReceived), 4);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->messagesHandled), 8);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->crcErrors), 12);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->handlerMicroseconds), 16);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->renderTime), 20);
    pack_int32_t(buffer, reinterpret_cast<const int32_t*>(&this->wifiRssi), 24);    
    return 28;
}

int MavLinkTelemetry::unpack(const char* buffer) {
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->messagesSent), 0);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->messagesReceived), 4);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->messagesHandled), 8);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->crcErrors), 12);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->handlerMicroseconds), 16);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->renderTime), 20);
    unpack_int32_t(buffer, reinterpret_cast<int32_t*>(&this->wifiRssi), 24);
    return 28;
}


std::string MavLinkMessageBase::char_array_tostring(int len, const char* field)
{
    int i = 0;
    for (i= 0; i < len; i++)
    {
        if (field[i] == '\0')
            break;
    }
    return std::string(field, i);
}

std::string MavLinkMessageBase::float_tostring(float value) 
{
    // json can't handle "nan", so we convert it to null.
    if (std::isnan(value) || std::isinf(value)) {
        return "null";
    }
    std::ostringstream s;
    s << value;
    return s.str();
}


template<class T>
class BinaryArray
{
public:
    static std::string toString(int len, const T* field) {
        std::ostringstream line;
        for (int i = 0; i < len; i++) {
            line << field[i];
            if (i + 1 < len) {
                line << ", ";
            }
        }
        return line.str();
    }
};

std::string MavLinkMessageBase::uint8_t_array_tostring(int len, const uint8_t* field)
{
    // ostringstream tries to convert uint8_t to 'char' which is not what we want here.
    std::ostringstream line;
    for (int i = 0; i < len; i++) {
        line << static_cast<unsigned int>(field[i]);
        if (i + 1 < len) {
            line << ", ";
        }
    }
    return line.str();
}
std::string MavLinkMessageBase::int8_t_array_tostring(int len, const int8_t* field)
{
    // ostringstream tries to convert int8_t to 'char' which is not what we want here.
    std::ostringstream line;
    for (int i = 0; i < len; i++) {
        line << static_cast<int>(field[i]);
        if (i + 1 < len) {
            line << ", ";
        }
    }
    return line.str();
}
std::string MavLinkMessageBase::int16_t_array_tostring(int len, const int16_t* field)
{
    return BinaryArray<int16_t>::toString(len, field);
}
std::string MavLinkMessageBase::uint16_t_array_tostring(int len, const uint16_t* field)
{
    return BinaryArray<uint16_t>::toString(len, field);
}

std::string MavLinkMessageBase::float_array_tostring(int len, const float* field)
{
    std::ostringstream line;
    for (int i = 0; i < len; i++) {
        line << float_tostring(field[i]);
        if (i + 1 < len) {
            line << ", ";
        }
    }
    return line.str();
}