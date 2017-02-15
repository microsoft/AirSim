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
# ifndef MSGPACK_PREPROCESSOR_DEBUG_ASSERT_HPP
# define MSGPACK_PREPROCESSOR_DEBUG_ASSERT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/expr_iif.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/logical/not.hpp>
# include <rpc/msgpack/preprocessor/tuple/eat.hpp>
#
# /* MSGPACK_PP_ASSERT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ASSERT MSGPACK_PP_ASSERT_D
# else
#    define MSGPACK_PP_ASSERT(cond) MSGPACK_PP_ASSERT_D(cond)
# endif
#
# define MSGPACK_PP_ASSERT_D(cond) MSGPACK_PP_IIF(MSGPACK_PP_NOT(cond), MSGPACK_PP_ASSERT_ERROR, MSGPACK_PP_TUPLE_EAT_1)(...)
# define MSGPACK_PP_ASSERT_ERROR(x, y, z)
#
# /* MSGPACK_PP_ASSERT_MSG */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ASSERT_MSG MSGPACK_PP_ASSERT_MSG_D
# else
#    define MSGPACK_PP_ASSERT_MSG(cond, msg) MSGPACK_PP_ASSERT_MSG_D(cond, msg)
# endif
#
# define MSGPACK_PP_ASSERT_MSG_D(cond, msg) MSGPACK_PP_EXPR_IIF(MSGPACK_PP_NOT(cond), msg)
#
# endif
