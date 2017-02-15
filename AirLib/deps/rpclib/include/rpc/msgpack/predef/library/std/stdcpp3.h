/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_STDCPP3_H
#define MSGPACK_PREDEF_LIBRARY_STD_STDCPP3_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_GNU`]

[@http://gcc.gnu.org/libstdc++/ GNU libstdc++] Standard C++ library.
Version number available as year (from 1970), month, and day.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__GLIBCXX__`] [__predef_detection__]]
    [[`__GLIBCPP__`] [__predef_detection__]]

    [[`__GLIBCXX__`] [V.R.P]]
    [[`__GLIBCPP__`] [V.R.P]]
    ]
 */

#define MSGPACK_LIB_STD_GNU MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GLIBCPP__) || defined(__GLIBCXX__)
#   undef MSGPACK_LIB_STD_GNU
#   if defined(__GLIBCXX__)
#       define MSGPACK_LIB_STD_GNU MSGPACK_PREDEF_MAKE_YYYYMMDD(__GLIBCXX__)
#   else
#       define MSGPACK_LIB_STD_GNU MSGPACK_PREDEF_MAKE_YYYYMMDD(__GLIBCPP__)
#   endif
#endif

#if MSGPACK_LIB_STD_GNU
#   define MSGPACK_LIB_STD_GNU_AVAILABLE
#endif

#define MSGPACK_LIB_STD_GNU_NAME "GNU"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_GNU,MSGPACK_LIB_STD_GNU_NAME)


#endif
