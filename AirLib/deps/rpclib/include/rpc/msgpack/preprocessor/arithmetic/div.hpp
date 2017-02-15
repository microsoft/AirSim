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
# ifndef MSGPACK_PREPROCESSOR_ARITHMETIC_DIV_HPP
# define MSGPACK_PREPROCESSOR_ARITHMETIC_DIV_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/detail/div_base.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_DIV */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_DIV(x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_DIV_BASE(x, y))
# else
#    define MSGPACK_PP_DIV(x, y) MSGPACK_PP_DIV_I(x, y)
#    define MSGPACK_PP_DIV_I(x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_DIV_BASE(x, y))
# endif
#
# /* MSGPACK_PP_DIV_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_DIV_D(d, x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_DIV_BASE_D(d, x, y))
# else
#    define MSGPACK_PP_DIV_D(d, x, y) MSGPACK_PP_DIV_D_I(d, x, y)
#    define MSGPACK_PP_DIV_D_I(d, x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_DIV_BASE_D(d, x, y))
# endif
#
# endif
