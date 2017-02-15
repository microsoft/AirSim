/*
Copyright Rene Rivera 2011-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_PYRAMID_H
#define MSGPACK_PREDEF_ARCHITECTURE_PYRAMID_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_ARCH_PYRAMID`]

Pyramid 9810 architecture.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`pyr`] [__predef_detection__]]
    ]
 */

#define MSGPACK_ARCH_PYRAMID MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(pyr)
#   undef MSGPACK_ARCH_PYRAMID
#   define MSGPACK_ARCH_PYRAMID MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_ARCH_PYRAMID
#   define MSGPACK_ARCH_PYRAMID_AVAILABLE
#endif

#define MSGPACK_ARCH_PYRAMID_NAME "Pyramid 9810"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_PYRAMID,MSGPACK_ARCH_PYRAMID_NAME)


#endif
