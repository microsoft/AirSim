/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_C_ZOS_H
#define MSGPACK_PREDEF_LIBRARY_C_ZOS_H

#include <rpc/msgpack/predef/library/c/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_C_ZOS`]

z/OS libc Standard C library.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__LIBREL__`] [__predef_detection__]]

    [[`__LIBREL__`] [V.R.P]]
    [[`__TARGET_LIB__`] [V.R.P]]
    ]
 */

#define MSGPACK_LIB_C_ZOS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__LIBREL__)
#   undef MSGPACK_LIB_C_ZOS
#   if !defined(MSGPACK_LIB_C_ZOS) && defined(__LIBREL__)
#       define MSGPACK_LIB_C_ZOS MSGPACK_PREDEF_MAKE_0X_VRRPPPP(__LIBREL__)
#   endif
#   if !defined(MSGPACK_LIB_C_ZOS) && defined(__TARGET_LIB__)
#       define MSGPACK_LIB_C_ZOS MSGPACK_PREDEF_MAKE_0X_VRRPPPP(__TARGET_LIB__)
#   endif
#   if !defined(MSGPACK_LIB_C_ZOS)
#       define MSGPACK_LIB_C_ZOS MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_LIB_C_ZOS
#   define MSGPACK_LIB_C_ZOS_AVAILABLE
#endif

#define MSGPACK_LIB_C_ZOS_NAME "z/OS"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_C_ZOS,MSGPACK_LIB_C_ZOS_NAME)


#endif
