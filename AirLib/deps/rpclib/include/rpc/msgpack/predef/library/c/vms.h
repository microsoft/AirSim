/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_C_VMS_H
#define MSGPACK_PREDEF_LIBRARY_C_VMS_H

#include <rpc/msgpack/predef/library/c/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_C_VMS`]

VMS libc Standard C library.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__CRTL_VER`] [__predef_detection__]]

    [[`__CRTL_VER`] [V.R.P]]
    ]
 */

#define MSGPACK_LIB_C_VMS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__CRTL_VER)
#   undef MSGPACK_LIB_C_VMS
#   define MSGPACK_LIB_C_VMS MSGPACK_PREDEF_MAKE_10_VVRR0PP00(__CRTL_VER)
#endif

#if MSGPACK_LIB_C_VMS
#   define MSGPACK_LIB_C_VMS_AVAILABLE
#endif

#define MSGPACK_LIB_C_VMS_NAME "VMS"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_C_VMS,MSGPACK_LIB_C_VMS_NAME)


#endif
