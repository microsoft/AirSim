/*
 * MessagePack system dependencies
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
#ifndef MSGPACK_SYSDEP_H
#define MSGPACK_SYSDEP_H

#include <rpc/msgpack/predef.h>

#include <stdlib.h>
#include <stddef.h>
#if defined(_MSC_VER) && _MSC_VER < 1600
    typedef __int8 int8_t;
    typedef unsigned __int8 uint8_t;
    typedef __int16 int16_t;
    typedef unsigned __int16 uint16_t;
    typedef __int32 int32_t;
    typedef unsigned __int32 uint32_t;
    typedef __int64 int64_t;
    typedef unsigned __int64 uint64_t;
#elif defined(_MSC_VER)  // && _MSC_VER >= 1600
#   include <stdint.h>
#else
#   include <stdint.h>
#   include <stdbool.h>
#endif

#if defined(_MSC_VER)
#   define MSGPACK_DLLEXPORT __declspec(dllexport)
#else  /* _MSC_VER */
#   define MSGPACK_DLLEXPORT
#endif /* _MSC_VER */

#ifdef _WIN32
#   define _msgpack_atomic_counter_header <windows.h>
    typedef long _msgpack_atomic_counter_t;
#   define _msgpack_sync_decr_and_fetch(ptr) InterlockedDecrement(ptr)
#   define _msgpack_sync_incr_and_fetch(ptr) InterlockedIncrement(ptr)
#elif defined(__GNUC__) && ((__GNUC__*10 + __GNUC_MINOR__) < 41)

#   if defined(__cplusplus)
#       define _msgpack_atomic_counter_header "gcc_atomic.hpp"
#   else
#       define _msgpack_atomic_counter_header "gcc_atomic.h"
#   endif

#else
    typedef unsigned int _msgpack_atomic_counter_t;
#   define _msgpack_sync_decr_and_fetch(ptr) __sync_sub_and_fetch(ptr, 1)
#   define _msgpack_sync_incr_and_fetch(ptr) __sync_add_and_fetch(ptr, 1)
#endif

#ifdef _WIN32

#   ifdef __cplusplus
    /* numeric_limits<T>::min,max */
#       ifdef max
#           undef max
#       endif
#       ifdef min
#           undef min
#       endif
#   endif

#else /* _*/

#include <arpa/inet.h>  /* __BYTE_ORDER */
#   if defined(linux)
#       include <byteswap.h>
#   endif

#endif

#ifdef MSGPACK_ENDIAN_LITTLE_BYTE

#   ifdef _WIN32
#       if defined(ntohs)
#       define _msgpack_be16(x) ntohs(x)
#        elif defined(_byteswap_ushort) || (defined(_MSC_VER) && _MSC_VER >= 1400)
#            define _msgpack_be16(x) ((uint16_t)_byteswap_ushort((unsigned short)x))
#        else
#            define _msgpack_be16(x) ( \
                 ((((uint16_t)x) <<  8) ) | \
                 ((((uint16_t)x) >>  8) ) )
#        endif
#   else
#        define _msgpack_be16(x) ntohs(x)
#   endif

#   ifdef _WIN32
#        if defined(ntohl)
#            define _msgpack_be32(x) ntohl(x)
#        elif defined(_byteswap_ulong) || (defined(_MSC_VER) && _MSC_VER >= 1400)
#            define _msgpack_be32(x) ((uint32_t)_byteswap_ulong((unsigned long)x))
#        else
#            define _msgpack_be32(x) \
                 ( ((((uint32_t)x) << 24)               ) | \
                   ((((uint32_t)x) <<  8) & 0x00ff0000U ) | \
                   ((((uint32_t)x) >>  8) & 0x0000ff00U ) | \
                   ((((uint32_t)x) >> 24)               ) )
#        endif
#   else
#        define _msgpack_be32(x) ntohl(x)
#   endif

#   if defined(_byteswap_uint64) || (defined(_MSC_VER) && _MSC_VER >= 1400)
#        define _msgpack_be64(x) (_byteswap_uint64(x))
#   elif defined(bswap_64)
#        define _msgpack_be64(x) bswap_64(x)
#   elif defined(__DARWIN_OSSwapInt64)
#        define _msgpack_be64(x) __DARWIN_OSSwapInt64(x)
#   else
#        define _msgpack_be64(x) \
             ( ((((uint64_t)x) << 56)                         ) | \
               ((((uint64_t)x) << 40) & 0x00ff000000000000ULL ) | \
               ((((uint64_t)x) << 24) & 0x0000ff0000000000ULL ) | \
               ((((uint64_t)x) <<  8) & 0x000000ff00000000ULL ) | \
               ((((uint64_t)x) >>  8) & 0x00000000ff000000ULL ) | \
               ((((uint64_t)x) >> 24) & 0x0000000000ff0000ULL ) | \
               ((((uint64_t)x) >> 40) & 0x000000000000ff00ULL ) | \
               ((((uint64_t)x) >> 56)                         ) )
#   endif

#elif MSGPACK_ENDIAN_BIG_BYTE

#   define _msgpack_be16(x) (x)
#   define _msgpack_be32(x) (x)
#   define _msgpack_be64(x) (x)

#else
#   error msgpack-c supports only big endian and little endian
#endif /* MSGPACK_ENDIAN_LITTLE_BYTE */

#define _msgpack_load16(cast, from, to) do {       \
        memcpy((cast*)(to), (from), sizeof(cast)); \
        *(to) = _msgpack_be16(*(to));              \
    } while (0);

#define _msgpack_load32(cast, from, to) do {       \
        memcpy((cast*)(to), (from), sizeof(cast)); \
        *(to) = _msgpack_be32(*(to));              \
    } while (0);
#define _msgpack_load64(cast, from, to) do {       \
        memcpy((cast*)(to), (from), sizeof(cast)); \
        *(to) = _msgpack_be64(*(to));              \
    } while (0);

#define _msgpack_store16(to, num) \
    do { uint16_t val = _msgpack_be16(num); memcpy(to, &val, 2); } while(0)
#define _msgpack_store32(to, num) \
    do { uint32_t val = _msgpack_be32(num); memcpy(to, &val, 4); } while(0)
#define _msgpack_store64(to, num) \
    do { uint64_t val = _msgpack_be64(num); memcpy(to, &val, 8); } while(0)

/*
#define _msgpack_load16(cast, from) \
    ({ cast val; memcpy(&val, (char*)from, 2); _msgpack_be16(val); })
#define _msgpack_load32(cast, from) \
    ({ cast val; memcpy(&val, (char*)from, 4); _msgpack_be32(val); })
#define _msgpack_load64(cast, from) \
    ({ cast val; memcpy(&val, (char*)from, 8); _msgpack_be64(val); })
*/


#if !defined(__cplusplus) && defined(_MSC_VER)
#  if !defined(FALSE)
#    define FALSE (0)
#  endif
#  if !defined(TRUE)
#    define TRUE (!FALSE)
#  endif
#  if _MSC_VER >= 1800
#    include <stdbool.h>
#  else
#    define bool int
#    define true TRUE
#    define false FALSE
#  endif
#  define inline __inline
#endif

#endif /* msgpack/sysdep.h */
