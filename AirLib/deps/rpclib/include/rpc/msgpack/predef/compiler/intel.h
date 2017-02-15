/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_INTEL_H
#define MSGPACK_PREDEF_COMPILER_INTEL_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_INTEL`]

[@http://en.wikipedia.org/wiki/Intel_C%2B%2B Intel C/C++] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__INTEL_COMPILER`] [__predef_detection__]]
    [[`__ICL`] [__predef_detection__]]
    [[`__ICC`] [__predef_detection__]]
    [[`__ECC`] [__predef_detection__]]

    [[`__INTEL_COMPILER`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_INTEL MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__INTEL_COMPILER) || defined(__ICL) || defined(__ICC) || \
    defined(__ECC)
#   if !defined(MSGPACK_COMP_INTEL_DETECTION) && defined(__INTEL_COMPILER)
#       define MSGPACK_COMP_INTEL_DETECTION MSGPACK_PREDEF_MAKE_10_VRP(__INTEL_COMPILER)
#   endif
#   if !defined(MSGPACK_COMP_INTEL_DETECTION)
#       define MSGPACK_COMP_INTEL_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_INTEL_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_INTEL_EMULATED MSGPACK_COMP_INTEL_DETECTION
#   else
#       undef MSGPACK_COMP_INTEL
#       define MSGPACK_COMP_INTEL MSGPACK_COMP_INTEL_DETECTION
#   endif
#   define MSGPACK_COMP_INTEL_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_INTEL_NAME "Intel C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_INTEL,MSGPACK_COMP_INTEL_NAME)

#ifdef MSGPACK_COMP_INTEL_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_INTEL_EMULATED,MSGPACK_COMP_INTEL_NAME)
#endif


#endif
