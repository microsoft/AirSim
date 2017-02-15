/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_OS_AIX_H
#define MSGPACK_PREDEF_OS_AIX_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_OS_AIX`]

[@http://en.wikipedia.org/wiki/AIX_operating_system IBM AIX] operating system.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`_AIX`] [__predef_detection__]]
    [[`__TOS_AIX__`] [__predef_detection__]]

    [[`_AIX43`] [4.3.0]]
    [[`_AIX41`] [4.1.0]]
    [[`_AIX32`] [3.2.0]]
    [[`_AIX3`] [3.0.0]]
    ]
 */

#define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(MSGPACK_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(_AIX) || defined(__TOS_AIX__) \
    )
#   undef MSGPACK_OS_AIX
#   if !defined(MSGPACK_OS_AIX) && defined(_AIX43)
#       define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER(4,3,0)
#   endif
#   if !defined(MSGPACK_OS_AIX) && defined(_AIX41)
#       define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER(4,1,0)
#   endif
#   if !defined(MSGPACK_OS_AIX) && defined(_AIX32)
#       define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER(3,2,0)
#   endif
#   if !defined(MSGPACK_OS_AIX) && defined(_AIX3)
#       define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER(3,0,0)
#   endif
#   if !defined(MSGPACK_OS_AIX)
#       define MSGPACK_OS_AIX MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_OS_AIX
#   define MSGPACK_OS_AIX_AVAILABLE
#   include <rpc/msgpack/predef/detail/os_detected.h>
#endif

#define MSGPACK_OS_AIX_NAME "IBM AIX"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_OS_AIX,MSGPACK_OS_AIX_NAME)


#endif
