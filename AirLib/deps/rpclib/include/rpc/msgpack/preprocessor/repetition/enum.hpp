# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_REPETITION_ENUM_HPP
# define MSGPACK_PREPROCESSOR_REPETITION_ENUM_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/debug/error.hpp>
# include <rpc/msgpack/preprocessor/detail/auto_rec.hpp>
# include <rpc/msgpack/preprocessor/punctuation/comma_if.hpp>
# include <rpc/msgpack/preprocessor/repetition/repeat.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_ENUM */
#
# if 0
#    define MSGPACK_PP_ENUM(count, macro, data)
# endif
#
# define MSGPACK_PP_ENUM MSGPACK_PP_CAT(MSGPACK_PP_ENUM_, MSGPACK_PP_AUTO_REC(MSGPACK_PP_REPEAT_P, 4))
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ENUM_1(c, m, d) MSGPACK_PP_REPEAT_1(c, MSGPACK_PP_ENUM_M_1, (m, d))
#    define MSGPACK_PP_ENUM_2(c, m, d) MSGPACK_PP_REPEAT_2(c, MSGPACK_PP_ENUM_M_2, (m, d))
#    define MSGPACK_PP_ENUM_3(c, m, d) MSGPACK_PP_REPEAT_3(c, MSGPACK_PP_ENUM_M_3, (m, d))
# else
#    define MSGPACK_PP_ENUM_1(c, m, d) MSGPACK_PP_ENUM_1_I(c, m, d)
#    define MSGPACK_PP_ENUM_2(c, m, d) MSGPACK_PP_ENUM_2_I(c, m, d)
#    define MSGPACK_PP_ENUM_3(c, m, d) MSGPACK_PP_ENUM_3_I(c, m, d)
#    define MSGPACK_PP_ENUM_1_I(c, m, d) MSGPACK_PP_REPEAT_1(c, MSGPACK_PP_ENUM_M_1, (m, d))
#    define MSGPACK_PP_ENUM_2_I(c, m, d) MSGPACK_PP_REPEAT_2(c, MSGPACK_PP_ENUM_M_2, (m, d))
#    define MSGPACK_PP_ENUM_3_I(c, m, d) MSGPACK_PP_REPEAT_3(c, MSGPACK_PP_ENUM_M_3, (m, d))
# endif
#
# define MSGPACK_PP_ENUM_4(c, m, d) MSGPACK_PP_ERROR(0x0003)
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_ENUM_M_1(z, n, md) MSGPACK_PP_ENUM_M_1_IM(z, n, MSGPACK_PP_TUPLE_REM_2 md)
#    define MSGPACK_PP_ENUM_M_2(z, n, md) MSGPACK_PP_ENUM_M_2_IM(z, n, MSGPACK_PP_TUPLE_REM_2 md)
#    define MSGPACK_PP_ENUM_M_3(z, n, md) MSGPACK_PP_ENUM_M_3_IM(z, n, MSGPACK_PP_TUPLE_REM_2 md)
#    define MSGPACK_PP_ENUM_M_1_IM(z, n, im) MSGPACK_PP_ENUM_M_1_I(z, n, im)
#    define MSGPACK_PP_ENUM_M_2_IM(z, n, im) MSGPACK_PP_ENUM_M_2_I(z, n, im)
#    define MSGPACK_PP_ENUM_M_3_IM(z, n, im) MSGPACK_PP_ENUM_M_3_I(z, n, im)
# else
#    define MSGPACK_PP_ENUM_M_1(z, n, md) MSGPACK_PP_ENUM_M_1_I(z, n, MSGPACK_PP_TUPLE_ELEM(2, 0, md), MSGPACK_PP_TUPLE_ELEM(2, 1, md))
#    define MSGPACK_PP_ENUM_M_2(z, n, md) MSGPACK_PP_ENUM_M_2_I(z, n, MSGPACK_PP_TUPLE_ELEM(2, 0, md), MSGPACK_PP_TUPLE_ELEM(2, 1, md))
#    define MSGPACK_PP_ENUM_M_3(z, n, md) MSGPACK_PP_ENUM_M_3_I(z, n, MSGPACK_PP_TUPLE_ELEM(2, 0, md), MSGPACK_PP_TUPLE_ELEM(2, 1, md))
# endif
#
# define MSGPACK_PP_ENUM_M_1_I(z, n, m, d) MSGPACK_PP_COMMA_IF(n) m(z, n, d)
# define MSGPACK_PP_ENUM_M_2_I(z, n, m, d) MSGPACK_PP_COMMA_IF(n) m(z, n, d)
# define MSGPACK_PP_ENUM_M_3_I(z, n, m, d) MSGPACK_PP_COMMA_IF(n) m(z, n, d)
#
# endif
