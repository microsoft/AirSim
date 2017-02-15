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
# ifndef MSGPACK_PREPROCESSOR_SELECTION_MIN_HPP
# define MSGPACK_PREPROCESSOR_SELECTION_MIN_HPP
#
# include <rpc/msgpack/preprocessor/comparison/less_equal.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
#
# /* MSGPACK_PP_MIN */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_MIN(x, y) MSGPACK_PP_IIF(MSGPACK_PP_LESS_EQUAL(y, x), y, x)
# else
#    define MSGPACK_PP_MIN(x, y) MSGPACK_PP_MIN_I(x, y)
#    define MSGPACK_PP_MIN_I(x, y) MSGPACK_PP_IIF(MSGPACK_PP_LESS_EQUAL(y, x), y, x)
# endif
#
# /* MSGPACK_PP_MIN_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_MIN_D(d, x, y) MSGPACK_PP_IIF(MSGPACK_PP_LESS_EQUAL_D(d, y, x), y, x)
# else
#    define MSGPACK_PP_MIN_D(d, x, y) MSGPACK_PP_MIN_D_I(d, x, y)
#    define MSGPACK_PP_MIN_D_I(d, x, y) MSGPACK_PP_IIF(MSGPACK_PP_LESS_EQUAL_D(d, y, x), y, x)
# endif
#
# endif
