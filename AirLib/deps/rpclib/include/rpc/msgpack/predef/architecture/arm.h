/*
Copyright Rene Rivera 2008-2013
Copyright Franz Detro 2014
Copyright (c) Microsoft Corporation 2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_ARM_H
#define MSGPACK_PREDEF_ARCHITECTURE_ARM_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_ARCH_ARM`]

[@http://en.wikipedia.org/wiki/ARM_architecture ARM] architecture.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__arm__`] [__predef_detection__]]
    [[`__arm64`] [__predef_detection__]]
    [[`__thumb__`] [__predef_detection__]]
    [[`__TARGET_ARCH_ARM`] [__predef_detection__]]
    [[`__TARGET_ARCH_THUMB`] [__predef_detection__]]
    [[`_M_ARM`] [__predef_detection__]]

    [[`__arm64`] [8.0.0]]
    [[`__TARGET_ARCH_ARM`] [V.0.0]]
    [[`__TARGET_ARCH_THUMB`] [V.0.0]]
    [[`_M_ARM`] [V.0.0]]
    ]
 */

#define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__arm__) || defined(__arm64) || defined(__thumb__) || \
    defined(__TARGET_ARCH_ARM) || defined(__TARGET_ARCH_THUMB) || \
    defined(_M_ARM)
#   undef MSGPACK_ARCH_ARM
#   if !defined(MSGPACK_ARCH_ARM) && defined(__arm64)
#       define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER(8,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_ARM) && defined(__TARGET_ARCH_ARM)
#       define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER(__TARGET_ARCH_ARM,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_ARM) && defined(__TARGET_ARCH_THUMB)
#       define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER(__TARGET_ARCH_THUMB,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_ARM) && defined(_M_ARM)
#       define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER(_M_ARM,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_ARM)
#       define MSGPACK_ARCH_ARM MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_ARCH_ARM
#   define MSGPACK_ARCH_ARM_AVAILABLE
#endif

#define MSGPACK_ARCH_ARM_NAME "ARM"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_ARM,MSGPACK_ARCH_ARM_NAME)


#endif
