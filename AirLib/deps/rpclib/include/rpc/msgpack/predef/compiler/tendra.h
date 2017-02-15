/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_TENDRA_H
#define MSGPACK_PREDEF_COMPILER_TENDRA_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_TENDRA`]

[@http://en.wikipedia.org/wiki/TenDRA_Compiler TenDRA C/C++] compiler.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__TenDRA__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_COMP_TENDRA MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__TenDRA__)
#   define MSGPACK_COMP_TENDRA_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#ifdef MSGPACK_COMP_TENDRA_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_TENDRA_EMULATED MSGPACK_COMP_TENDRA_DETECTION
#   else
#       undef MSGPACK_COMP_TENDRA
#       define MSGPACK_COMP_TENDRA MSGPACK_COMP_TENDRA_DETECTION
#   endif
#   define MSGPACK_COMP_TENDRA_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_TENDRA_NAME "TenDRA C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_TENDRA,MSGPACK_COMP_TENDRA_NAME)

#ifdef MSGPACK_COMP_TENDRA_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_TENDRA_EMULATED,MSGPACK_COMP_TENDRA_NAME)
#endif


#endif
