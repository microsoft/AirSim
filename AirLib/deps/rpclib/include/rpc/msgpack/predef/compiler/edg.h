/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_EDG_H
#define MSGPACK_PREDEF_COMPILER_EDG_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_EDG`]

[@http://en.wikipedia.org/wiki/Edison_Design_Group EDG C++ Frontend] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__EDG__`] [__predef_detection__]]

    [[`__EDG_VERSION__`] [V.R.0]]
    ]
 */

#define MSGPACK_COMP_EDG MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__EDG__)
#   define MSGPACK_COMP_EDG_DETECTION MSGPACK_PREDEF_MAKE_10_VRR(__EDG_VERSION__)
#endif

#ifdef MSGPACK_COMP_EDG_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_EDG_EMULATED MSGPACK_COMP_EDG_DETECTION
#   else
#       undef MSGPACK_COMP_EDG
#       define MSGPACK_COMP_EDG MSGPACK_COMP_EDG_DETECTION
#   endif
#   define MSGPACK_COMP_EDG_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_EDG_NAME "EDG C++ Frontend"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_EDG,MSGPACK_COMP_EDG_NAME)

#ifdef MSGPACK_COMP_EDG_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_EDG_EMULATED,MSGPACK_COMP_EDG_NAME)
#endif


#endif
