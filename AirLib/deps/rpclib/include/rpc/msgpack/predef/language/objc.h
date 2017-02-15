/*
Copyright Rene Rivera 2011-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LANGUAGE_OBJC_H
#define MSGPACK_PREDEF_LANGUAGE_OBJC_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LANG_OBJC`]

[@http://en.wikipedia.org/wiki/Objective-C Objective-C] language.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__OBJC__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_LANG_OBJC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__OBJC__)
#   undef MSGPACK_LANG_OBJC
#   define MSGPACK_LANG_OBJC MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_LANG_OBJC
#   define MSGPACK_LANG_OBJC_AVAILABLE
#endif

#define MSGPACK_LANG_OBJC_NAME "Objective-C"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LANG_OBJC,MSGPACK_LANG_OBJC_NAME)


#endif
