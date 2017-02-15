/*
Copyright Rene Rivera 2011-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_CXX_H
#define MSGPACK_PREDEF_LIBRARY_STD_CXX_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_CXX`]

[@http://libcxx.llvm.org/ libc++] C++ Standard Library.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`_LIBCPP_VERSION`] [__predef_detection__]]

    [[`_LIBCPP_VERSION`] [V.0.P]]
    ]
 */

#define MSGPACK_LIB_STD_CXX MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(_LIBCPP_VERSION)
#   undef MSGPACK_LIB_STD_CXX
#   define MSGPACK_LIB_STD_CXX MSGPACK_PREDEF_MAKE_10_VPPP(_LIBCPP_VERSION)
#endif

#if MSGPACK_LIB_STD_CXX
#   define MSGPACK_LIB_STD_CXX_AVAILABLE
#endif

#define MSGPACK_LIB_STD_CXX_NAME "libc++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_CXX,MSGPACK_LIB_STD_CXX_NAME)


#endif
