/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_X86_H
#define MSGPACK_PREDEF_ARCHITECTURE_X86_H

#include <rpc/msgpack/predef/architecture/x86/32.h>
#include <rpc/msgpack/predef/architecture/x86/64.h>

/*`
[heading `MSGPACK_ARCH_X86`]

[@http://en.wikipedia.org/wiki/X86 Intel x86] architecture. This is
a category to indicate that either `MSGPACK_ARCH_X86_32` or
`MSGPACK_ARCH_X86_64` is detected.
 */

#define MSGPACK_ARCH_X86 MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if MSGPACK_ARCH_X86_32 || MSGPACK_ARCH_X86_64
#   undef MSGPACK_ARCH_X86
#   define MSGPACK_ARCH_X86 MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_ARCH_X86
#   define MSGPACK_ARCH_X86_AVAILABLE
#endif

#define MSGPACK_ARCH_X86_NAME "Intel x86"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_X86,MSGPACK_ARCH_X86_NAME)

#endif
