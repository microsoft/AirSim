/*
Copyright Rene Rivera 2008-2013
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_LIBRARY_STD_STLPORT_H
#define MSGPACK_PREDEF_LIBRARY_STD_STLPORT_H

#include <rpc/msgpack/predef/library/std/_prefix.h>

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_LIB_STD_STLPORT`]

[@http://sourceforge.net/projects/stlport/ STLport Standard C++] library.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__SGI_STL_PORT`] [__predef_detection__]]
    [[`_STLPORT_VERSION`] [__predef_detection__]]

    [[`_STLPORT_MAJOR`, `_STLPORT_MINOR`, `_STLPORT_PATCHLEVEL`] [V.R.P]]
    [[`_STLPORT_VERSION`] [V.R.P]]
    [[`__SGI_STL_PORT`] [V.R.P]]
    ]
 */

#define MSGPACK_LIB_STD_STLPORT MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)
#   undef MSGPACK_LIB_STD_STLPORT
#   if !defined(MSGPACK_LIB_STD_STLPORT) && defined(_STLPORT_MAJOR)
#       define MSGPACK_LIB_STD_STLPORT \
            MSGPACK_VERSION_NUMBER(_STLPORT_MAJOR,_STLPORT_MINOR,_STLPORT_PATCHLEVEL)
#   endif
#   if !defined(MSGPACK_LIB_STD_STLPORT) && defined(_STLPORT_VERSION)
#       define MSGPACK_LIB_STD_STLPORT MSGPACK_PREDEF_MAKE_0X_VRP(_STLPORT_VERSION)
#   endif
#   if !defined(MSGPACK_LIB_STD_STLPORT)
#       define MSGPACK_LIB_STD_STLPORT MSGPACK_PREDEF_MAKE_0X_VRP(__SGI_STL_PORT)
#   endif
#endif

#if MSGPACK_LIB_STD_STLPORT
#   define MSGPACK_LIB_STD_STLPORT_AVAILABLE
#endif

#define MSGPACK_LIB_STD_STLPORT_NAME "STLport"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_LIB_STD_STLPORT,MSGPACK_LIB_STD_STLPORT_NAME)


#endif
