/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_IBM_H
#define MSGPACK_PREDEF_COMPILER_IBM_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_IBM`]

[@http://en.wikipedia.org/wiki/VisualAge IBM XL C/C++] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__IBMCPP__`] [__predef_detection__]]
    [[`__xlC__`] [__predef_detection__]]
    [[`__xlc__`] [__predef_detection__]]

    [[`__COMPILER_VER__`] [V.R.P]]
    [[`__xlC__`] [V.R.P]]
    [[`__xlc__`] [V.R.P]]
    [[`__IBMCPP__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_IBM MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IBMCPP__) || defined(__xlC__) || defined(__xlc__)
#   if !defined(MSGPACK_COMP_IBM_DETECTION) && defined(__COMPILER_VER__)
#       define MSGPACK_COMP_IBM_DETECTION MSGPACK_PREDEF_MAKE_0X_VRRPPPP(__COMPILER_VER__)
#   endif
#   if !defined(MSGPACK_COMP_IBM_DETECTION) && defined(__xlC__)
#       define MSGPACK_COMP_IBM_DETECTION MSGPACK_PREDEF_MAKE_0X_VVRR(__xlC__)
#   endif
#   if !defined(MSGPACK_COMP_IBM_DETECTION) && defined(__xlc__)
#       define MSGPACK_COMP_IBM_DETECTION MSGPACK_PREDEF_MAKE_0X_VVRR(__xlc__)
#   endif
#   if !defined(MSGPACK_COMP_IBM_DETECTION)
#       define MSGPACK_COMP_IBM_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(__IBMCPP__)
#   endif
#endif

#ifdef MSGPACK_COMP_IBM_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_IBM_EMULATED MSGPACK_COMP_IBM_DETECTION
#   else
#       undef MSGPACK_COMP_IBM
#       define MSGPACK_COMP_IBM MSGPACK_COMP_IBM_DETECTION
#   endif
#   define MSGPACK_COMP_IBM_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_IBM_NAME "IBM XL C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_IBM,MSGPACK_COMP_IBM_NAME)

#ifdef MSGPACK_COMP_IBM_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_IBM_EMULATED,MSGPACK_COMP_IBM_NAME)
#endif


#endif
