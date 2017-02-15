/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_COMEAU_H
#define MSGPACK_PREDEF_COMPILER_COMEAU_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

#define MSGPACK_COMP_COMO MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

/*`
[heading `MSGPACK_COMP_COMO`]

[@http://en.wikipedia.org/wiki/Comeau_C/C%2B%2B Comeau C++] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__COMO__`] [__predef_detection__]]

    [[`__COMO_VERSION__`] [V.R.P]]
    ]
 */

#if defined(__COMO__)
#   if !defined(MSGPACK_COMP_COMO_DETECTION) && defined(__CONO_VERSION__)
#       define MSGPACK_COMP_COMO_DETECTION MSGPACK_PREDEF_MAKE_0X_VRP(__COMO_VERSION__)
#   endif
#   if !defined(MSGPACK_COMP_COMO_DETECTION)
#       define MSGPACK_COMP_COMO_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_COMO_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_COMO_EMULATED MSGPACK_COMP_COMO_DETECTION
#   else
#       undef MSGPACK_COMP_COMO
#       define MSGPACK_COMP_COMO MSGPACK_COMP_COMO_DETECTION
#   endif
#   define MSGPACK_COMP_COMO_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_COMO_NAME "Comeau C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_COMO,MSGPACK_COMP_COMO_NAME)

#ifdef MSGPACK_COMP_COMO_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_COMO_EMULATED,MSGPACK_COMP_COMO_NAME)
#endif


#endif
