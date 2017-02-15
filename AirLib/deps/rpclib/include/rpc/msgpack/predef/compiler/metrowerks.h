/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_METROWERKS_H
#define MSGPACK_PREDEF_COMPILER_METROWERKS_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_MWERKS`]

[@http://en.wikipedia.org/wiki/CodeWarrior Metrowerks CodeWarrior] compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__MWERKS__`] [__predef_detection__]]
    [[`__CWCC__`] [__predef_detection__]]

    [[`__CWCC__`] [V.R.P]]
    [[`__MWERKS__`] [V.R.P >= 4.2.0]]
    [[`__MWERKS__`] [9.R.0]]
    [[`__MWERKS__`] [8.R.0]]
    ]
 */

#define MSGPACK_COMP_MWERKS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__MWERKS__) || defined(__CWCC__)
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION) && defined(__CWCC__)
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_PREDEF_MAKE_0X_VRPP(__CWCC__)
#   endif
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION) && (__MWERKS__ >= 0x4200)
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_PREDEF_MAKE_0X_VRPP(__MWERKS__)
#   endif
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION) && (__MWERKS__ >= 0x3204) // note the "skip": 04->9.3
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_VERSION_NUMBER(9,(__MWERKS__)%100-1,0)
#   endif
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION) && (__MWERKS__ >= 0x3200)
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_VERSION_NUMBER(9,(__MWERKS__)%100,0)
#   endif
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION) && (__MWERKS__ >= 0x3000)
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_VERSION_NUMBER(8,(__MWERKS__)%100,0)
#   endif
#   if !defined(MSGPACK_COMP_MWERKS_DETECTION)
#       define MSGPACK_COMP_MWERKS_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_MWERKS_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_MWERKS_EMULATED MSGPACK_COMP_MWERKS_DETECTION
#   else
#       undef MSGPACK_COMP_MWERKS
#       define MSGPACK_COMP_MWERKS MSGPACK_COMP_MWERKS_DETECTION
#   endif
#   define MSGPACK_COMP_MWERKS_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_MWERKS_NAME "Metrowerks CodeWarrior"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_MWERKS,MSGPACK_COMP_MWERKS_NAME)

#ifdef MSGPACK_COMP_MWERKS_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_MWERKS_EMULATED,MSGPACK_COMP_MWERKS_NAME)
#endif


#endif
