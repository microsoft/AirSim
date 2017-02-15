/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_SUNPRO_H
#define MSGPACK_PREDEF_COMPILER_SUNPRO_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_SUNPRO`]

[@http://en.wikipedia.org/wiki/Sun_Studio_%28software%29 Sun Studio] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__SUNPRO_CC`] [__predef_detection__]]
    [[`__SUNPRO_C`] [__predef_detection__]]

    [[`__SUNPRO_CC`] [V.R.P]]
    [[`__SUNPRO_C`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_SUNPRO MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SUNPRO_CC) || defined(__SUNPRO_C)
#   if !defined(MSGPACK_COMP_SUNPRO_DETECTION) && defined(__SUNPRO_CC)
#       define MSGPACK_COMP_SUNPRO_DETECTION MSGPACK_PREDEF_MAKE_0X_VRP(__SUNPRO_CC)
#   endif
#   if !defined(MSGPACK_COMP_SUNPRO_DETECTION) && defined(__SUNPRO_C)
#       define MSGPACK_COMP_SUNPRO_DETECTION MSGPACK_PREDEF_MAKE_0X_VRP(__SUNPRO_C)
#   endif
#   if !defined(MSGPACK_COMP_SUNPRO_DETECTION)
#       define MSGPACK_COMP_SUNPRO_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_SUNPRO_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_SUNPRO_EMULATED MSGPACK_COMP_SUNPRO_DETECTION
#   else
#       undef MSGPACK_COMP_SUNPRO
#       define MSGPACK_COMP_SUNPRO MSGPACK_COMP_SUNPRO_DETECTION
#   endif
#   define MSGPACK_COMP_SUNPRO_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_SUNPRO_NAME "Sun Studio"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_SUNPRO,MSGPACK_COMP_SUNPRO_NAME)

#ifdef MSGPACK_COMP_SUNPRO_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_SUNPRO_EMULATED,MSGPACK_COMP_SUNPRO_NAME)
#endif


#endif
