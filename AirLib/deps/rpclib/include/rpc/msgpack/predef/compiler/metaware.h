/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_METAWARE_H
#define MSGPACK_PREDEF_COMPILER_METAWARE_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_HIGHC`]

MetaWare High C/C++ compiler.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__HIGHC__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_COMP_HIGHC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__HIGHC__)
#   define MSGPACK_COMP_HIGHC_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#ifdef MSGPACK_COMP_HIGHC_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_HIGHC_EMULATED MSGPACK_COMP_HIGHC_DETECTION
#   else
#       undef MSGPACK_COMP_HIGHC
#       define MSGPACK_COMP_HIGHC MSGPACK_COMP_HIGHC_DETECTION
#   endif
#   define MSGPACK_COMP_HIGHC_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_HIGHC_NAME "MetaWare High C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_HIGHC,MSGPACK_COMP_HIGHC_NAME)

#ifdef MSGPACK_COMP_HIGHC_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_HIGHC_EMULATED,MSGPACK_COMP_HIGHC_NAME)
#endif


#endif
