/*
Copyright Jessica Hamilton 2014
Copyright Rene Rivera 2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_OS_HAIKU_H
#define MSGPACK_PREDEF_OS_HAIKU_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_OS_HAIKU`]

[@http://en.wikipedia.org/wiki/Haiku_(operating_system) Haiku] operating system.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__HAIKU__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_OS_HAIKU MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(MSGPACK_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__HAIKU__) \
    )
#   undef MSGPACK_OS_HAIKU
#   define MSGPACK_OS_HAIKU MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_OS_HAIKU
#   define MSGPACK_OS_HAIKU_AVAILABLE
#   include <rpc/msgpack/predef/detail/os_detected.h>
#endif

#define MSGPACK_OS_HAIKU_NAME "Haiku"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_OS_HAIKU,MSGPACK_OS_HAIKU_NAME)


#endif
