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
# ifndef MSGPACK_PREPROCESSOR_REPETITION_REPEAT_FROM_TO_HPP
# define MSGPACK_PREPROCESSOR_REPETITION_REPEAT_FROM_TO_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/add.hpp>
# include <rpc/msgpack/preprocessor/arithmetic/sub.hpp>
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/debug/error.hpp>
# include <rpc/msgpack/preprocessor/detail/auto_rec.hpp>
# include <rpc/msgpack/preprocessor/repetition/repeat.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_REPEAT_FROM_TO */
#
# if 0
#    define MSGPACK_PP_REPEAT_FROM_TO(first, last, macro, data)
# endif
#
# define MSGPACK_PP_REPEAT_FROM_TO MSGPACK_PP_CAT(MSGPACK_PP_REPEAT_FROM_TO_, MSGPACK_PP_AUTO_REC(MSGPACK_PP_REPEAT_P, 4))
#
# define MSGPACK_PP_REPEAT_FROM_TO_1(f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_1(MSGPACK_PP_AUTO_REC(MSGPACK_PP_WHILE_P, 256), f, l, m, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_2(f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_2(MSGPACK_PP_AUTO_REC(MSGPACK_PP_WHILE_P, 256), f, l, m, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_3(f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_3(MSGPACK_PP_AUTO_REC(MSGPACK_PP_WHILE_P, 256), f, l, m, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_4(f, l, m, dt) MSGPACK_PP_ERROR(0x0003)
#
# define MSGPACK_PP_REPEAT_FROM_TO_1ST MSGPACK_PP_REPEAT_FROM_TO_1
# define MSGPACK_PP_REPEAT_FROM_TO_2ND MSGPACK_PP_REPEAT_FROM_TO_2
# define MSGPACK_PP_REPEAT_FROM_TO_3RD MSGPACK_PP_REPEAT_FROM_TO_3
#
# /* MSGPACK_PP_REPEAT_FROM_TO_D */
#
# if 0
#    define MSGPACK_PP_REPEAT_FROM_TO_D(d, first, last, macro, data)
# endif
#
# define MSGPACK_PP_REPEAT_FROM_TO_D MSGPACK_PP_CAT(MSGPACK_PP_REPEAT_FROM_TO_D_, MSGPACK_PP_AUTO_REC(MSGPACK_PP_REPEAT_P, 4))
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_REPEAT_FROM_TO_D_1(d, f, l, m, dt) MSGPACK_PP_REPEAT_1(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_1, (d, f, m, dt))
#    define MSGPACK_PP_REPEAT_FROM_TO_D_2(d, f, l, m, dt) MSGPACK_PP_REPEAT_2(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_2, (d, f, m, dt))
#    define MSGPACK_PP_REPEAT_FROM_TO_D_3(d, f, l, m, dt) MSGPACK_PP_REPEAT_3(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_3, (d, f, m, dt))
# else
#    define MSGPACK_PP_REPEAT_FROM_TO_D_1(d, f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_1_I(d, f, l, m, dt)
#    define MSGPACK_PP_REPEAT_FROM_TO_D_2(d, f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_2_I(d, f, l, m, dt)
#    define MSGPACK_PP_REPEAT_FROM_TO_D_3(d, f, l, m, dt) MSGPACK_PP_REPEAT_FROM_TO_D_3_I(d, f, l, m, dt)
#    define MSGPACK_PP_REPEAT_FROM_TO_D_1_I(d, f, l, m, dt) MSGPACK_PP_REPEAT_1(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_1, (d, f, m, dt))
#    define MSGPACK_PP_REPEAT_FROM_TO_D_2_I(d, f, l, m, dt) MSGPACK_PP_REPEAT_2(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_2, (d, f, m, dt))
#    define MSGPACK_PP_REPEAT_FROM_TO_D_3_I(d, f, l, m, dt) MSGPACK_PP_REPEAT_3(MSGPACK_PP_SUB_D(d, l, f), MSGPACK_PP_REPEAT_FROM_TO_M_3, (d, f, m, dt))
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_REPEAT_FROM_TO_M_1(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_1_IM(z, n, MSGPACK_PP_TUPLE_REM_4 dfmd)
#    define MSGPACK_PP_REPEAT_FROM_TO_M_2(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_2_IM(z, n, MSGPACK_PP_TUPLE_REM_4 dfmd)
#    define MSGPACK_PP_REPEAT_FROM_TO_M_3(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_3_IM(z, n, MSGPACK_PP_TUPLE_REM_4 dfmd)
#    define MSGPACK_PP_REPEAT_FROM_TO_M_1_IM(z, n, im) MSGPACK_PP_REPEAT_FROM_TO_M_1_I(z, n, im)
#    define MSGPACK_PP_REPEAT_FROM_TO_M_2_IM(z, n, im) MSGPACK_PP_REPEAT_FROM_TO_M_2_I(z, n, im)
#    define MSGPACK_PP_REPEAT_FROM_TO_M_3_IM(z, n, im) MSGPACK_PP_REPEAT_FROM_TO_M_3_I(z, n, im)
# else
#    define MSGPACK_PP_REPEAT_FROM_TO_M_1(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_1_I(z, n, MSGPACK_PP_TUPLE_ELEM(4, 0, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 1, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 2, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 3, dfmd))
#    define MSGPACK_PP_REPEAT_FROM_TO_M_2(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_2_I(z, n, MSGPACK_PP_TUPLE_ELEM(4, 0, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 1, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 2, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 3, dfmd))
#    define MSGPACK_PP_REPEAT_FROM_TO_M_3(z, n, dfmd) MSGPACK_PP_REPEAT_FROM_TO_M_3_I(z, n, MSGPACK_PP_TUPLE_ELEM(4, 0, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 1, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 2, dfmd), MSGPACK_PP_TUPLE_ELEM(4, 3, dfmd))
# endif
#
# define MSGPACK_PP_REPEAT_FROM_TO_M_1_I(z, n, d, f, m, dt) MSGPACK_PP_REPEAT_FROM_TO_M_1_II(z, MSGPACK_PP_ADD_D(d, n, f), m, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_M_2_I(z, n, d, f, m, dt) MSGPACK_PP_REPEAT_FROM_TO_M_2_II(z, MSGPACK_PP_ADD_D(d, n, f), m, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_M_3_I(z, n, d, f, m, dt) MSGPACK_PP_REPEAT_FROM_TO_M_3_II(z, MSGPACK_PP_ADD_D(d, n, f), m, dt)
#
# define MSGPACK_PP_REPEAT_FROM_TO_M_1_II(z, n, m, dt) m(z, n, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_M_2_II(z, n, m, dt) m(z, n, dt)
# define MSGPACK_PP_REPEAT_FROM_TO_M_3_II(z, n, m, dt) m(z, n, dt)
#
# endif
