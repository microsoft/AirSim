/*
Copyright Rene Rivera 2011-2012
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LANGUAGE_STDC_H
#define MSGPACK_PREDEF_LANGUAGE_STDC_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LANG_STDC`]

[@http://en.wikipedia.org/wiki/C_(programming_language) Standard C] language.
If available, the year of the standard is detected as YYYY.MM.1 from the Epoc date.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__STDC__`] [__predef_detection__]]

    [[`__STDC_VERSION__`] [V.R.P]]
    ]
 */

#define MSGPACK_LANG_STDC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__STDC__)
#   undef MSGPACK_LANG_STDC
#   if defined(__STDC_VERSION__)
#       if (__STDC_VERSION__ > 100)
#           define MSGPACK_LANG_STDC MSGPACK_PREDEF_MAKE_YYYYMM(__STDC_VERSION__)
#       else
#           define MSGPACK_LANG_STDC MSGPACK_VERSION_NUMBER_AVAILABLE
#       endif
#   else
#       define MSGPACK_LANG_STDC MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if MSGPACK_LANG_STDC
#   define MSGPACK_LANG_STDC_AVAILABLE
#endif

#define MSGPACK_LANG_STDC_NAME "Standard C"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LANG_STDC,MSGPACK_LANG_STDC_NAME)


#endif
