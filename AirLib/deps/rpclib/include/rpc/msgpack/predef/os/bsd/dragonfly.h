/*
Copyright Rene Rivera 2012-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_OS_BSD_DRAGONFLY_H
#define MSGPACK_PREDEF_OS_BSD_DRAGONFLY_H

#include <rpc/msgpack/predef/os/bsd.h>

/*`
[heading `MSGPACK_OS_BSD_DRAGONFLY`]

[@http://en.wikipedia.org/wiki/DragonFly_BSD DragonFly BSD] operating system.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__DragonFly__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_OS_BSD_DRAGONFLY MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(MSGPACK_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__DragonFly__) \
    )
#   ifndef MSGPACK_OS_BSD_AVAILABLE
#       define MSGPACK_OS_BSD MSGPACK_VERSION_NUMBER_AVAILABLE
#       define MSGPACK_OS_BSD_AVAILABLE
#   endif
#   undef MSGPACK_OS_BSD_DRAGONFLY
#   if defined(__DragonFly__)
#       define MSGPACK_OS_DRAGONFLY_BSD MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_OS_BSD_DRAGONFLY
#   define MSGPACK_OS_BSD_DRAGONFLY_AVAILABLE
#   include <rpc/msgpack/predef/detail/os_detected.h>
#endif

#define MSGPACK_OS_BSD_DRAGONFLY_NAME "DragonFly BSD"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_OS_BSD_DRAGONFLY,MSGPACK_OS_BSD_DRAGONFLY_NAME)

#endif
