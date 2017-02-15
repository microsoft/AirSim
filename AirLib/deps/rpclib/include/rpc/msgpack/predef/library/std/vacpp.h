/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_VACPP_H
#define MSGPACK_PREDEF_LIBRARY_STD_VACPP_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_IBM`]

[@http://www.ibm.com/software/awdtools/xlcpp/ IBM VACPP Standard C++] library.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__IBMCPP__`] [__predef_detection__]]
    ]
 */

#define MSGPACK_LIB_STD_IBM MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IBMCPP__)
#   undef MSGPACK_LIB_STD_IBM
#   define MSGPACK_LIB_STD_IBM MSGPACK_VERSION_NUMBER_AVAILABLE
#endif

#if MSGPACK_LIB_STD_IBM
#   define MSGPACK_LIB_STD_IBM_AVAILABLE
#endif

#define MSGPACK_LIB_STD_IBM_NAME "IBM VACPP"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_IBM,MSGPACK_LIB_STD_IBM_NAME)


#endif
