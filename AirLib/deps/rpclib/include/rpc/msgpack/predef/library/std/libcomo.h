/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_LIBCOMO_H
#define MSGPACK_PREDEF_LIBRARY_STD_LIBCOMO_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_COMO`]

[@http://www.comeaucomputing.com/libcomo/ Comeau Computing] Standard C++ Library.
Version number available as major.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__LIBCOMO__`] [__predef_detection__]]

    [[`__LIBCOMO_VERSION__`] [V.0.0]]
    ]
 */

#define MSGPACK_LIB_STD_COMO MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__LIBCOMO__)
#   undef MSGPACK_LIB_STD_COMO
#   define MSGPACK_LIB_STD_COMO MSGPACK_VERSION_NUMBER(__LIBCOMO_VERSION__,0,0)
#endif

#if MSGPACK_LIB_STD_COMO
#   define MSGPACK_LIB_STD_COMO_AVAILABLE
#endif

#define MSGPACK_LIB_STD_COMO_NAME "Comeau Computing"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_COMO,MSGPACK_LIB_STD_COMO_NAME)


#endif
