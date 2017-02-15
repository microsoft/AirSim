/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_CLANG_H
#define MSGPACK_PREDEF_COMPILER_CLANG_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_CLANG`]

[@http://en.wikipedia.org/wiki/Clang Clang] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__clang__`] [__predef_detection__]]

    [[`__clang_major__`, `__clang_minor__`, `__clang_patchlevel__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_CLANG MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__clang__)
#   define MSGPACK_COMP_CLANG_DETECTION MSGPACK_VERSION_NUMBER(__clang_major__,__clang_minor__,__clang_patchlevel__)
#endif

#ifdef MSGPACK_COMP_CLANG_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_CLANG_EMULATED MSGPACK_COMP_CLANG_DETECTION
#   else
#       undef MSGPACK_COMP_CLANG
#       define MSGPACK_COMP_CLANG MSGPACK_COMP_CLANG_DETECTION
#   endif
#   define MSGPACK_COMP_CLANG_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_CLANG_NAME "Clang"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_CLANG,MSGPACK_COMP_CLANG_NAME)

#ifdef MSGPACK_COMP_CLANG_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_CLANG_EMULATED,MSGPACK_COMP_CLANG_NAME)
#endif


#endif
