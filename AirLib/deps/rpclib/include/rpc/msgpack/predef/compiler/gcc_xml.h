/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_GCC_XML_H
#define MSGPACK_PREDEF_COMPILER_GCC_XML_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_GCCXML`]

[@http://www.gccxml.org/ GCC XML] compiler.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__GCCXML__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_COMP_GCCXML MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GCCXML__)
#   define MSGPACK_COMP_GCCXML_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#ifdef MSGPACK_COMP_GCCXML_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_GCCXML_EMULATED MSGPACK_COMP_GCCXML_DETECTION
#   else
#       undef MSGPACK_COMP_GCCXML
#       define MSGPACK_COMP_GCCXML MSGPACK_COMP_GCCXML_DETECTION
#   endif
#   define MSGPACK_COMP_GCCXML_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_GCCXML_NAME "GCC XML"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GCCXML,MSGPACK_COMP_GCCXML_NAME)

#ifdef MSGPACK_COMP_GCCXML_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_GCCXML_EMULATED,MSGPACK_COMP_GCCXML_NAME)
#endif

#endif
