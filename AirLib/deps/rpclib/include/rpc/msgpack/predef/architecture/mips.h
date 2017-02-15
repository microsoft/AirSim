/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_MIPS_H
#define MSGPACK_PREDEF_ARCHITECTURE_MIPS_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_ARCH_MIPS`]

[@http://en.wikipedia.org/wiki/MIPS_architecture MIPS] architecture.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__mips__`] [__predef_detection__]]
    [[`__mips`] [__predef_detection__]]
    [[`__MIPS__`] [__predef_detection__]]

    [[`__mips`] [V.0.0]]
    [[`_MIPS_ISA_MIPS1`] [1.0.0]]
    [[`_R3000`] [1.0.0]]
    [[`_MIPS_ISA_MIPS2`] [2.0.0]]
    [[`__MIPS_ISA2__`] [2.0.0]]
    [[`_R4000`] [2.0.0]]
    [[`_MIPS_ISA_MIPS3`] [3.0.0]]
    [[`__MIPS_ISA3__`] [3.0.0]]
    [[`_MIPS_ISA_MIPS4`] [4.0.0]]
    [[`__MIPS_ISA4__`] [4.0.0]]
    ]
 */

#define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__mips__) || defined(__mips) || \
    defined(__MIPS__)
#   undef MSGPACK_ARCH_MIPS
#   if !defined(MSGPACK_ARCH_MIPS) && (defined(__mips))
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER(__mips,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS1) || defined(_R3000))
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER(1,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS2) || defined(__MIPS_ISA2__) || defined(_R4000))
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER(2,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS3) || defined(__MIPS_ISA3__))
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER(3,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS4) || defined(__MIPS_ISA4__))
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER(4,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_MIPS)
#       define MSGPACK_ARCH_MIPS MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_ARCH_MIPS
#   define MSGPACK_ARCH_MIPS_AVAILABLE
#endif

#define MSGPACK_ARCH_MIPS_NAME "MIPS"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_MIPS,MSGPACK_ARCH_MIPS_NAME)


#endif
