/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_GREENHILLS_H
#define MSGPACK_PREDEF_COMPILER_GREENHILLS_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_GHS`]

[@http://en.wikipedia.org/wiki/Green_Hills_Software Green Hills C/C++] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__ghs`] [__predef_detection__]]
    [[`__ghs__`] [__predef_detection__]]

    [[`__GHS_VERSION_NUMBER__`] [V.R.P]]
    [[`__ghs`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_GHS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__ghs) || defined(__ghs__)
#   if !defined(MSGPACK_COMP_GHS_DETECTION) && defined(__GHS_VERSION_NUMBER__)
#       define MSGPACK_COMP_GHS_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(__GHS_VERSION_NUMBER__)
#   endif
#   if !defined(MSGPACK_COMP_GHS_DETECTION) && defined(__ghs)
#       define MSGPACK_COMP_GHS_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(__ghs)
#   endif
#   if !defined(MSGPACK_COMP_GHS_DETECTION)
#       define MSGPACK_COMP_GHS_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_GHS_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_GHS_EMULATED MSGPACK_COMP_GHS_DETECTION
#   else
#       undef MSGPACK_COMP_GHS
#       define MSGPACK_COMP_GHS MSGPACK_COMP_GHS_DETECTION
#   endif
#   define MSGPACK_COMP_GHS_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_GHS_NAME "Green Hills C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GHS,MSGPACK_COMP_GHS_NAME)

#ifdef MSGPACK_COMP_GHS_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GHS_EMULATED,MSGPACK_COMP_GHS_NAME)
#endif


#endif
