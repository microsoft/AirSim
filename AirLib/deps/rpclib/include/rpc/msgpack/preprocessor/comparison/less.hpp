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
# ifndef MSGPACK_PREPROCESSOR_COMPARISON_LESS_HPP
# define MSGPACK_PREPROCESSOR_COMPARISON_LESS_HPP
#
# include <rpc/msgpack/preprocessor/comparison/less_equal.hpp>
# include <rpc/msgpack/preprocessor/comparison/not_equal.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/logical/bitand.hpp>
# include <rpc/msgpack/preprocessor/tuple/eat.hpp>
#
# /* MSGPACK_PP_LESS */
#
# if MSGPACK_PP_CONFIG_FLAGS() & (MSGPACK_PP_CONFIG_MWCC() | MSGPACK_PP_CONFIG_DMC())
#    define MSGPACK_PP_LESS(x, y) MSGPACK_PP_BITAND(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL(x, y))
# elif ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LESS(x, y) MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL, 0 MSGPACK_PP_TUPLE_EAT_2)(x, y)
# else
#    define MSGPACK_PP_LESS(x, y) MSGPACK_PP_LESS_I(x, y)
#    define MSGPACK_PP_LESS_I(x, y) MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL, 0 MSGPACK_PP_TUPLE_EAT_2)(x, y)
# endif
#
# /* MSGPACK_PP_LESS_D */
#
# if MSGPACK_PP_CONFIG_FLAGS() & (MSGPACK_PP_CONFIG_MWCC() | MSGPACK_PP_CONFIG_DMC())
#    define MSGPACK_PP_LESS_D(d, x, y) MSGPACK_PP_BITAND(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL_D(d, x, y))
# elif ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LESS_D(d, x, y) MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL_D, 0 MSGPACK_PP_TUPLE_EAT_3)(d, x, y)
# else
#    define MSGPACK_PP_LESS_D(d, x, y) MSGPACK_PP_LESS_D_I(d, x, y)
#    define MSGPACK_PP_LESS_D_I(d, x, y) MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(x, y), MSGPACK_PP_LESS_EQUAL_D, 0 MSGPACK_PP_TUPLE_EAT_3)(d, x, y)
# endif
#
# endif
