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
# ifndef MSGPACK_PREPROCESSOR_COMPARISON_LESS_EQUAL_HPP
# define MSGPACK_PREPROCESSOR_COMPARISON_LESS_EQUAL_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/sub.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/logical/not.hpp>
#
# /* MSGPACK_PP_LESS_EQUAL */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LESS_EQUAL(x, y) MSGPACK_PP_NOT(MSGPACK_PP_SUB(x, y))
# else
#    define MSGPACK_PP_LESS_EQUAL(x, y) MSGPACK_PP_LESS_EQUAL_I(x, y)
#    define MSGPACK_PP_LESS_EQUAL_I(x, y) MSGPACK_PP_NOT(MSGPACK_PP_SUB(x, y))
# endif
#
# /* MSGPACK_PP_LESS_EQUAL_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LESS_EQUAL_D(d, x, y) MSGPACK_PP_NOT(MSGPACK_PP_SUB_D(d, x, y))
# else
#    define MSGPACK_PP_LESS_EQUAL_D(d, x, y) MSGPACK_PP_LESS_EQUAL_D_I(d, x, y)
#    define MSGPACK_PP_LESS_EQUAL_D_I(d, x, y) MSGPACK_PP_NOT(MSGPACK_PP_SUB_D(d, x, y))
# endif
#
# endif
