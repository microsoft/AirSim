/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_GCC_H
#define MSGPACK_PREDEF_COMPILER_GCC_H

/* Other compilers that emulate this one need to be detected first. */

#include <rpc/msgpack/predef/compiler/clang.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_GNUC`]

[@http://en.wikipedia.org/wiki/GNU_Compiler_Collection Gnu GCC C/C++] compiler.
Version number available as major, minor, and patch (if available).

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__GNUC__`] [__predef_detection__]]

    [[`__GNUC__`, `__GNUC_MINOR__`, `__GNUC_PATCHLEVEL__`] [V.R.P]]
    [[`__GNUC__`, `__GNUC_MINOR__`] [V.R.0]]
    ]
 */

#define MSGPACK_COMP_GNUC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GNUC__)
#   if !defined(MSGPACK_COMP_GNUC_DETECTION) && defined(__GNUC_PATCHLEVEL__)
#       define MSGPACK_COMP_GNUC_DETECTION \
            MSGPACK_VERSION_NUMBER(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__)
#   endif
#   if !defined(MSGPACK_COMP_GNUC_DETECTION)
#       define MSGPACK_COMP_GNUC_DETECTION \
            MSGPACK_VERSION_NUMBER(__GNUC__,__GNUC_MINOR__,0)
#   endif
#endif

#ifdef MSGPACK_COMP_GNUC_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_GNUC_EMULATED MSGPACK_COMP_GNUC_DETECTION
#   else
#       undef MSGPACK_COMP_GNUC
#       define MSGPACK_COMP_GNUC MSGPACK_COMP_GNUC_DETECTION
#   endif
#   define MSGPACK_COMP_GNUC_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_GNUC_NAME "Gnu GCC C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GNUC,MSGPACK_COMP_GNUC_NAME)

#ifdef MSGPACK_COMP_GNUC_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GNUC_EMULATED,MSGPACK_COMP_GNUC_NAME)
#endif


#endif
