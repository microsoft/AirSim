/*
Copyright Rene Rivera 2011-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_ARCHITECTURE_CONVEX_H
#define MSGPACK_PREDEF_ARCHITECTURE_CONVEX_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_ARCH_CONVEX`]

[@http://en.wikipedia.org/wiki/Convex_Computer Convex Computer] architecture.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__convex__`] [__predef_detection__]]

    [[`__convex_c1__`] [1.0.0]]
    [[`__convex_c2__`] [2.0.0]]
    [[`__convex_c32__`] [3.2.0]]
    [[`__convex_c34__`] [3.4.0]]
    [[`__convex_c38__`] [3.8.0]]
    ]
 */

#define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__convex__)
#   undef MSGPACK_ARCH_CONVEX
#   if !defined(MSGPACK_ARCH_CONVEX) && defined(__convex_c1__)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER(1,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_CONVEX) && defined(__convex_c2__)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER(2,0,0)
#   endif
#   if !defined(MSGPACK_ARCH_CONVEX) && defined(__convex_c32__)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER(3,2,0)
#   endif
#   if !defined(MSGPACK_ARCH_CONVEX) && defined(__convex_c34__)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER(3,4,0)
#   endif
#   if !defined(MSGPACK_ARCH_CONVEX) && defined(__convex_c38__)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER(3,8,0)
#   endif
#   if !defined(MSGPACK_ARCH_CONVEX)
#       define MSGPACK_ARCH_CONVEX MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_ARCH_CONVEX
#   define MSGPACK_ARCH_CONVEX_AVAILABLE
#endif

#define MSGPACK_ARCH_CONVEX_NAME "Convex Computer"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_ARCH_CONVEX,MSGPACK_ARCH_CONVEX_NAME)



#endif
