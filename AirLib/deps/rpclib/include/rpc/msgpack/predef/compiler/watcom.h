/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_WATCOM_H
#define MSGPACK_PREDEF_COMPILER_WATCOM_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_WATCOM`]

[@http://en.wikipedia.org/wiki/Watcom Watcom C++] compiler.
Version number available as major, and minor.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__WATCOMC__`] [__predef_detection__]]

    [[`__WATCOMC__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_WATCOM MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__WATCOMC__)
#   define MSGPACK_COMP_WATCOM_DETECTION MSGPACK_PREDEF_MAKE_10_VVRR(__WATCOMC__)
#endif

#ifdef MSGPACK_COMP_WATCOM_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_WATCOM_EMULATED MSGPACK_COMP_WATCOM_DETECTION
#   else
#       undef MSGPACK_COMP_WATCOM
#       define MSGPACK_COMP_WATCOM MSGPACK_COMP_WATCOM_DETECTION
#   endif
#   define MSGPACK_COMP_WATCOM_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_WATCOM_NAME "Watcom C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_WATCOM,MSGPACK_COMP_WATCOM_NAME)

#ifdef MSGPACK_COMP_WATCOM_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_WATCOM_EMULATED,MSGPACK_COMP_WATCOM_NAME)
#endif


#endif
