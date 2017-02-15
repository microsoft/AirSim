/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_DINKUMWARE_H
#define MSGPACK_PREDEF_LIBRARY_STD_DINKUMWARE_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_DINKUMWARE`]

[@http://en.wikipedia.org/wiki/Dinkumware Dinkumware] Standard C++ Library.
If available version number as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`_YVALS`, `__IBMCPP__`] [__predef_detection__]]
    [[`_CPPLIB_VER`] [__predef_detection__]]

    [[`_CPPLIB_VER`] [V.R.0]]
    ]
 */

#define MSGPACK_LIB_STD_DINKUMWARE MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if (defined(_YVALS) && !defined(__IBMCPP__)) || defined(_CPPLIB_VER)
#   undef MSGPACK_LIB_STD_DINKUMWARE
#   if defined(_CPPLIB_VER)
#       define MSGPACK_LIB_STD_DINKUMWARE MSGPACK_PREDEF_MAKE_10_VVRR(_CPPLIB_VER)
#   else
#       define MSGPACK_LIB_STD_DINKUMWARE MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_LIB_STD_DINKUMWARE
#   define MSGPACK_LIB_STD_DINKUMWARE_AVAILABLE
#endif

#define MSGPACK_LIB_STD_DINKUMWARE_NAME "Dinkumware"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_DINKUMWARE,MSGPACK_LIB_STD_DINKUMWARE_NAME)


#endif
