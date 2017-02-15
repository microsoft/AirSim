/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_SGI_MIPSPRO_H
#define MSGPACK_PREDEF_COMPILER_SGI_MIPSPRO_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_SGI`]

[@http://en.wikipedia.org/wiki/MIPSpro SGI MIPSpro] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__sgi`] [__predef_detection__]]
    [[`sgi`] [__predef_detection__]]

    [[`_SGI_COMPILER_VERSION`] [V.R.P]]
    [[`_COMPILER_VERSION`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_SGI MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__sgi) || defined(sgi)
#   if !defined(MSGPACK_COMP_SGI_DETECTION) && defined(_SGI_COMPILER_VERSION)
#       define MSGPACK_COMP_SGI_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(_SGI_COMPILER_VERSION)
#   endif
#   if !defined(MSGPACK_COMP_SGI_DETECTION) && defined(_COMPILER_VERSION)
#       define MSGPACK_COMP_SGI_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(_COMPILER_VERSION)
#   endif
#   if !defined(MSGPACK_COMP_SGI_DETECTION)
#       define MSGPACK_COMP_SGI_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_SGI_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_SGI_EMULATED MSGPACK_COMP_SGI_DETECTION
#   else
#       undef MSGPACK_COMP_SGI
#       define MSGPACK_COMP_SGI MSGPACK_COMP_SGI_DETECTION
#   endif
#   define MSGPACK_COMP_SGI_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_SGI_NAME "SGI MIPSpro"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_SGI,MSGPACK_COMP_SGI_NAME)

#ifdef MSGPACK_COMP_SGI_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_SGI_EMULATED,MSGPACK_COMP_SGI_NAME)
#endif


#endif
