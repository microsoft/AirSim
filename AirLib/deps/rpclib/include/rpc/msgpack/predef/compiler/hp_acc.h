/*
Copyright Rene Rivera 2008-2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef MSGPACK_PREDEF_COMPILER_HP_ACC_H
#define MSGPACK_PREDEF_COMPILER_HP_ACC_H

#include <rpc/msgpack/predef/version_number.h>
#include <rpc/msgpack/predef/make.h>

/*`
[heading `MSGPACK_COMP_HPACC`]

HP aC++ compiler.
Version number available as major, minor, and patch.

[table
    [[__predef_symbol__] [__predef_version__]]

    [[`__HP_aCC`] [__predef_detection__]]

    [[`__HP_aCC`] [V.R.P]]
    ]
 */

#define MSGPACK_COMP_HPACC MSGPACK_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__HP_aCC)
#   if !defined(MSGPACK_COMP_HPACC_DETECTION) && (__HP_aCC > 1)
#       define MSGPACK_COMP_HPACC_DETECTION MSGPACK_PREDEF_MAKE_10_VVRRPP(__HP_aCC)
#   endif
#   if !defined(MSGPACK_COMP_HPACC_DETECTION)
#       define MSGPACK_COMP_HPACC_DETECTION MSGPACK_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef MSGPACK_COMP_HPACC_DETECTION
#   if defined(MSGPACK_PREDEF_DETAIL_COMP_DETECTED)
#       define MSGPACK_COMP_HPACC_EMULATED MSGPACK_COMP_HPACC_DETECTION
#   else
#       undef MSGPACK_COMP_HPACC
#       define MSGPACK_COMP_HPACC MSGPACK_COMP_HPACC_DETECTION
#   endif
#   define MSGPACK_COMP_HPACC_AVAILABLE
#   include <rpc/msgpack/predef/detail/comp_detected.h>
#endif

#define MSGPACK_COMP_HPACC_NAME "HP aC++"

#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_HPACC,MSGPACK_COMP_HPACC_NAME)

#ifdef MSGPACK_COMP_HPACC_EMULATED
#include <rpc/msgpack/predef/detail/test.h>
MSGPACK_PREDEF_DECLARE_TEST(MSGPACK_COMP_HPACC_EMULATED,MSGPACK_COMP_HPACC_NAME)
#endif


#endif
