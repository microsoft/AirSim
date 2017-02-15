/*
 * MessagePack unpacking routine template
 *
 * Copyright (C) 2008-2010 FURUHASHI Sadayuki
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#ifndef MSGPACK_UNPACK_DEFINE_H
#define MSGPACK_UNPACK_DEFINE_H

#include "rpc/msgpack/sysdep.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifndef MSGPACK_EMBED_STACK_SIZE
#define MSGPACK_EMBED_STACK_SIZE 32
#endif


typedef enum {
    MSGPACK_CS_HEADER            = 0x00,  // nil

    //MSGPACK_CS_                = 0x01,
    //MSGPACK_CS_                = 0x02,  // false
    //MSGPACK_CS_                = 0x03,  // true

    MSGPACK_CS_BIN_8             = 0x04,
    MSGPACK_CS_BIN_16            = 0x05,
    MSGPACK_CS_BIN_32            = 0x06,

    MSGPACK_CS_EXT_8             = 0x07,
    MSGPACK_CS_EXT_16            = 0x08,
    MSGPACK_CS_EXT_32            = 0x09,

    MSGPACK_CS_FLOAT             = 0x0a,
    MSGPACK_CS_DOUBLE            = 0x0b,
    MSGPACK_CS_UINT_8            = 0x0c,
    MSGPACK_CS_UINT_16           = 0x0d,
    MSGPACK_CS_UINT_32           = 0x0e,
    MSGPACK_CS_UINT_64           = 0x0f,
    MSGPACK_CS_INT_8             = 0x10,
    MSGPACK_CS_INT_16            = 0x11,
    MSGPACK_CS_INT_32            = 0x12,
    MSGPACK_CS_INT_64            = 0x13,

    MSGPACK_CS_FIXEXT_1          = 0x14,
    MSGPACK_CS_FIXEXT_2          = 0x15,
    MSGPACK_CS_FIXEXT_4          = 0x16,
    MSGPACK_CS_FIXEXT_8          = 0x17,
    MSGPACK_CS_FIXEXT_16         = 0x18,

    MSGPACK_CS_STR_8             = 0x19, // str8
    MSGPACK_CS_STR_16            = 0x1a, // str16
    MSGPACK_CS_STR_32            = 0x1b, // str32
    MSGPACK_CS_ARRAY_16          = 0x1c,
    MSGPACK_CS_ARRAY_32          = 0x1d,
    MSGPACK_CS_MAP_16            = 0x1e,
    MSGPACK_CS_MAP_32            = 0x1f,

    //MSGPACK_ACS_BIG_INT_VALUE,
    //MSGPACK_ACS_BIG_FLOAT_VALUE,
    MSGPACK_ACS_STR_VALUE,
    MSGPACK_ACS_BIN_VALUE,
    MSGPACK_ACS_EXT_VALUE
} msgpack_unpack_state;


typedef enum {
    MSGPACK_CT_ARRAY_ITEM,
    MSGPACK_CT_MAP_KEY,
    MSGPACK_CT_MAP_VALUE
} msgpack_container_type;


#ifdef __cplusplus
}
#endif

#endif /* msgpack/unpack_define.h */

