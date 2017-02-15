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
# ifndef MSGPACK_PREPROCESSOR_ARITHMETIC_DETAIL_DIV_BASE_HPP
# define MSGPACK_PREPROCESSOR_ARITHMETIC_DETAIL_DIV_BASE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/arithmetic/sub.hpp>
# include <rpc/msgpack/preprocessor/comparison/less_equal.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_DIV_BASE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_DIV_BASE(x, y) MSGPACK_PP_WHILE(MSGPACK_PP_DIV_BASE_P, MSGPACK_PP_DIV_BASE_O, (0, x, y))
# else
#    define MSGPACK_PP_DIV_BASE(x, y) MSGPACK_PP_DIV_BASE_I(x, y)
#    define MSGPACK_PP_DIV_BASE_I(x, y) MSGPACK_PP_WHILE(MSGPACK_PP_DIV_BASE_P, MSGPACK_PP_DIV_BASE_O, (0, x, y))
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_DIV_BASE_P(d, rxy) MSGPACK_PP_DIV_BASE_P_IM(d, MSGPACK_PP_TUPLE_REM_3 rxy)
#    define MSGPACK_PP_DIV_BASE_P_IM(d, im) MSGPACK_PP_DIV_BASE_P_I(d, im)
# else
#    define MSGPACK_PP_DIV_BASE_P(d, rxy) MSGPACK_PP_DIV_BASE_P_I(d, MSGPACK_PP_TUPLE_ELEM(3, 0, rxy), MSGPACK_PP_TUPLE_ELEM(3, 1, rxy), MSGPACK_PP_TUPLE_ELEM(3, 2, rxy))
# endif
#
# define MSGPACK_PP_DIV_BASE_P_I(d, r, x, y) MSGPACK_PP_LESS_EQUAL_D(d, y, x)
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_DIV_BASE_O(d, rxy) MSGPACK_PP_DIV_BASE_O_IM(d, MSGPACK_PP_TUPLE_REM_3 rxy)
#    define MSGPACK_PP_DIV_BASE_O_IM(d, im) MSGPACK_PP_DIV_BASE_O_I(d, im)
# else
#    define MSGPACK_PP_DIV_BASE_O(d, rxy) MSGPACK_PP_DIV_BASE_O_I(d, MSGPACK_PP_TUPLE_ELEM(3, 0, rxy), MSGPACK_PP_TUPLE_ELEM(3, 1, rxy), MSGPACK_PP_TUPLE_ELEM(3, 2, rxy))
# endif
#
# define MSGPACK_PP_DIV_BASE_O_I(d, r, x, y) (MSGPACK_PP_INC(r), MSGPACK_PP_SUB_D(d, x, y), y)
#
# /* MSGPACK_PP_DIV_BASE_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_DIV_BASE_D(d, x, y) MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_DIV_BASE_P, MSGPACK_PP_DIV_BASE_O, (0, x, y))
# else
#    define MSGPACK_PP_DIV_BASE_D(d, x, y) MSGPACK_PP_DIV_BASE_D_I(d, x, y)
#    define MSGPACK_PP_DIV_BASE_D_I(d, x, y) MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_DIV_BASE_P, MSGPACK_PP_DIV_BASE_O, (0, x, y))
# endif
#
# endif
