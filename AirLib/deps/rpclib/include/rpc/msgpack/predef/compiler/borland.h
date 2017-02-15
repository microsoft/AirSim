/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_BORLAND_H
#define MSGPACK_PREDEF_COMPILER_BORLAND_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_BORLAND`]

[@http://en.wikipedia.org/wiki/C_plus_plus_builder Borland C++] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__BORLANDC__`] [__predef_detection__]]
    [[`__CODEGEARC__`] [__predef_detection__]]

    [[`__BORLANDC__`] [V.R.P]]
    [[`__CODEGEARC__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_BORLAND MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__BORLANDC__) || defined(__CODEGEARC__)
#   if !defined(MSGPACK_COMP_BORLAND_DETECTION) && (defined(__CODEGEARC__))
#       define MSGPACK_COMP_BORLAND_DETECTION MSGPACK_PREDEF_MAKE_0X_VVRP(__CODEGEARC__)
#   endif
#   if !defined(MSGPACK_COMP_BORLAND_DETECTION)
#       define MSGPACK_COMP_BORLAND_DETECTION MSGPACK_PREDEF_MAKE_0X_VVRP(__BORLANDC__)
#   endif
#endif

#ifdef MSGPACK_COMP_BORLAND_DETECTION
#   define MSGPACK_COMP_BORLAND_AVAILABLE
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_BORLAND_EMULATED MSGPACK_COMP_BORLAND_DETECTION
#   else
#       undef MSGPACK_COMP_BORLAND
#       define MSGPACK_COMP_BORLAND MSGPACK_COMP_BORLAND_DETECTION
#   endif
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_BORLAND_NAME "Borland C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_BORLAND,MSGPACK_COMP_BORLAND_NAME)

#ifdef MSGPACK_COMP_BORLAND_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_BORLAND_EMULATED,MSGPACK_COMP_BORLAND_NAME)
#endif


#endif
