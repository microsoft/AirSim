/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_DIGITALMARS_H
#define MSGPACK_PREDEF_COMPILER_DIGITALMARS_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_DMC`]

[@http://en.wikipedia.org/wiki/Digital_Mars Digital Mars] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__DMC__`] [__predef_detection__]]

    [[`__DMC__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_DMC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__DMC__)
#   define MSGPACK_COMP_DMC_DETECTION MSGPACK_PREDEF_MAKE_0X_VRP(__DMC__)
#endif

#ifdef MSGPACK_COMP_DMC_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_DMC_EMULATED MSGPACK_COMP_DMC_DETECTION
#   else
#       undef MSGPACK_COMP_DMC
#       define MSGPACK_COMP_DMC MSGPACK_COMP_DMC_DETECTION
#   endif
#   define MSGPACK_COMP_DMC_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_DMC_NAME "Digital Mars"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_DMC,MSGPACK_COMP_DMC_NAME)

#ifdef MSGPACK_COMP_DMC_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_DMC_EMULATED,MSGPACK_COMP_DMC_NAME)
#endif


#endif
