/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_IAR_H
#define MSGPACK_PREDEF_COMPILER_IAR_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_IAR`]

IAR C/C++ compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__IAR_SYSTEMS_ICC__`] [__predef_detection__]]

    [[`__VER__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_IAR MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IAR_SYSTEMS_ICC__)
#   define MSGPACK_COMP_IAR_DETECTION MSGPACK_PREDEF_MAKE_10_VVRR(__VER__)
#endif

#ifdef MSGPACK_COMP_IAR_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_IAR_EMULATED MSGPACK_COMP_IAR_DETECTION
#   else
#       undef MSGPACK_COMP_IAR
#       define MSGPACK_COMP_IAR MSGPACK_COMP_IAR_DETECTION
#   endif
#   define MSGPACK_COMP_IAR_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_IAR_NAME "IAR C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_IAR,MSGPACK_COMP_IAR_NAME)

#ifdef MSGPACK_COMP_IAR_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_IAR_EMULATED,MSGPACK_COMP_IAR_NAME)
#endif


#endif
