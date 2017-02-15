/*
Copyright Rene Rivera 2011-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_OS_VMS_H
#define MSGPACK_PREDEF_OS_VMS_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_OS_VMS`]

[@http://en.wikipedia.org/wiki/Vms VMS] operating system.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`VMS`] [__predef_detection__]]
    [[`__VMS`] [__predef_detection__]]

    [[`__VMS_VER`] [V.R.P]]
    ]
 */

#define MSGPACK_OS_VMS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(MSGPACK_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(VMS) || defined(__VMS) \
    )
#   undef MSGPACK_OS_VMS
#   if defined(__VMS_VER)
#       define MSGPACK_OS_VMS MSGPACK_PREDEF_MAKE_10_VVRR00PP00(__VMS_VER)
#   else
#       define MSGPACK_OS_VMS MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_OS_VMS
#   define MSGPACK_OS_VMS_AVAILABLE
#   include <rpc/msgpack/predef/detail/os_detected.h>
#endif

#define MSGPACK_OS_VMS_NAME "VMS"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_OS_VMS,MSGPACK_OS_VMS_NAME)


#endif
