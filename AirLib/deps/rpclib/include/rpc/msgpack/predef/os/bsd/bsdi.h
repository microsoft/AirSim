/*
Copyright Rene Rivera 2012-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_OS_BSD_BSDI_H
#define MSGPACK_PREDEF_OS_BSD_BSDI_H

#include <rpc/msgpack/predef/os/bsd.h>

/*`
[heading `MSGPACK_OS_BSD_BSDI`]

[@http://en.wikipedia.org/wiki/BSD/OS BSDi BSD/OS] operating system.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__bsdi__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_OS_BSD_BSDI MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(MSGPACK_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__bsdi__) \
    )
#   ifndef MSGPACK_OS_BSD_AVAILABLE
#       define MSGPACK_OS_BSD MSGPACK_VERSION_NUMBER_AVAILABLE
#       define MSGPACK_OS_BSD_AVAILABLE
#   endif
#   undef MSGPACK_OS_BSD_BSDI
#   define MSGPACK_OS_BSD_BSDI MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_OS_BSD_BSDI
#   define MSGPACK_OS_BSD_BSDI_AVAILABLE
#   include <rpc/msgpack/predef/detail/os_detected.h>
#endif

#define MSGPACK_OS_BSD_BSDI_NAME "BSDi BSD/OS"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_OS_BSD_BSDI,MSGPACK_OS_BSD_BSDI_NAME)

#endif
