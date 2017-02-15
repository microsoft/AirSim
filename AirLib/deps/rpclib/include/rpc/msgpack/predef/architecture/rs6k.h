/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_RS6K_H
#define MSGPACK_PREDEF_ARCHITECTURE_RS6K_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_ARCH_RS6000`]

[@http://en.wikipedia.org/wiki/RS/6000 RS/6000] architecture.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__THW_RS6000`] [__predef_detection__]]
    [[`_IBMR2`] [__predef_detection__]]
    [[`_POWER`] [__predef_detection__]]
    [[`_ARCH_PWR`] [__predef_detection__]]
    [[`_ARCH_PWR2`] [__predef_detection__]]
    ]
 */

#define MSGPACK_ARCH_RS6000 MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__THW_RS6000) || defined(_IBMR2) || \
    defined(_POWER) || defined(_ARCH_PWR) || \
    defined(_ARCH_PWR2)
#   undef MSGPACK_ARCH_RS6000
#   define MSGPACK_ARCH_RS6000 MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_ARCH_RS6000
#   define MSGPACK_ARCH_RS6000_AVAILABLE
#endif

#define MSGPACK_ARCH_RS6000_NAME "RS/6000"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_RS6000,MSGPACK_ARCH_RS6000_NAME)

#define MSGPACK_ARCH_PWR MSGPACK_ARCH_RS6000

#if MSGPACK_ARCH_PWR
#   define MSGPACK_ARCH_PWR_AVAILABLE
#endif

#define MSGPACK_ARCH_PWR_NAME MSGPACK_ARCH_RS6000_NAME

#endif
