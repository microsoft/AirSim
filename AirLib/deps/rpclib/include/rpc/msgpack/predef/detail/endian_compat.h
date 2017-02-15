/*
Copyright Rene Rivera 2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_DETAIL_ENDIAN_COMPAT_H
#define MSGPACK_PREDEF_DETAIL_ENDIAN_COMPAT_H

#include <rpc/msgpack/predef/other/endian.h>

#if MSGPACK_ENDIAN_BIG_BYTE
#   define MSGPACK_BIG_ENDIAN
#   define MSGPACK_BYTE_ORDER 4321
#endif
#if MSGPACK_ENDIAN_LITTLE_BYTE
#   define MSGPACK_LITTLE_ENDIAN
#   define MSGPACK_BYTE_ORDER 1234
#endif
#if MSGPACK_ENDIAN_LITTLE_WORD
#   define MSGPACK_PDP_ENDIAN
#   define MSGPACK_BYTE_ORDER 2134
#endif

#endif
