/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_C_UC_H
#define MSGPACK_PREDEF_LIBRARY_C_UC_H

#include <rpc/msgpack/predef/library/c/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_C_UC`]

[@http://en.wikipedia.org/wiki/Uclibc uClibc] Standard C library.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__UCLIBC__`] [__predef_detection__]]

    [[`__UCLIBC_MAJOR__`, `__UCLIBC_MINOR__`, `__UCLIBC_SUBLEVEL__`] [V.R.P]]
    ]
 */

#define MSGPACK_LIB_C_UC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__UCLIBC__)
#   undef MSGPACK_LIB_C_UC
#   define MSGPACK_LIB_C_UC MSGPACK_VERSION_NUMBER(\
        __UCLIBC_MAJOR__,__UCLIBC_MINOR__,__UCLIBC_SUBLEVEL__)
#endif

#if MSGPACK_LIB_C_UC
#   define MSGPACK_LIB_C_UC_AVAILABLE
#endif

#define MSGPACK_LIB_C_UC_NAME "uClibc"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_C_UC,MSGPACK_LIB_C_UC_NAME)


#endif
