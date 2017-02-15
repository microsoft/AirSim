/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_PGI_H
#define MSGPACK_PREDEF_COMPILER_PGI_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_PGI`]

[@http://en.wikipedia.org/wiki/The_Portland_Group Portland Group C/C++] compiler.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__PGI`] [__predef_detection__]]

    [[`__PGIC__`, `__PGIC_MINOR__`, `__PGIC_PATCHLEVEL__`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_PGI MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__PGI)
#   if !defined(MSGPACK_COMP_PGI_DETECTION) && (defined(__PGIC__) && defined(__PGIC_MINOR__) && defined(__PGIC_PATCHLEVEL__))
#       define MSGPACK_COMP_PGI_DETECTION MSGPACK_VERSION_NUMBER(__PGIC__,__PGIC_MINOR__,__PGIC_PATCHLEVEL__)
#   endif
#   if !defined(MSGPACK_COMP_PGI_DETECTION)
#       define MSGPACK_COMP_PGI_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_PGI_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_PGI_EMULATED MSGPACK_COMP_PGI_DETECTION
#   else
#       undef MSGPACK_COMP_PGI
#       define MSGPACK_COMP_PGI MSGPACK_COMP_PGI_DETECTION
#   endif
#   define MSGPACK_COMP_PGI_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_PGI_NAME "Portland Group C/C++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_PGI,MSGPACK_COMP_PGI_NAME)

#ifdef MSGPACK_COMP_PGI_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_PGI_EMULATED,MSGPACK_COMP_PGI_NAME)
#endif


#endif
