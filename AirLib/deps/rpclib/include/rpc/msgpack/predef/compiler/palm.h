/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_PALM_H
#define MSGPACK_PREDEF_COMPILER_PALM_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_PALM`]

Palm C/C++ compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`_PACC_VER`] [__predef_detection__]]

    [[`_PACC_VER`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_PALM MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(_PACC_VER)
#   define MSGPACK_COMP_PALM_DETECTION MSGPACK_PREDEF_MAKE_0X_VRRPP000(_PACC_VER)
#endif

#ifdef MSGPACK_COMP_PALM_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_PALM_EMULATED MSGPACK_COMP_PALM_DETECTION
#   else
#       undef MSGPACK_COMP_PALM
#       define MSGPACK_COMP_PALM MSGPACK_COMP_PALM_DETECTION
#   endif
#   define MSGPACK_COMP_PALM_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_PALM_NAME "Palm C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_PALM,MSGPACK_COMP_PALM_NAME)

#ifdef MSGPACK_COMP_PALM_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_PALM_EMULATED,MSGPACK_COMP_PALM_NAME)
#endif


#endif
