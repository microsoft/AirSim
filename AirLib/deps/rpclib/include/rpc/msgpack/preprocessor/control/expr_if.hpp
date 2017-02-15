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
# ifndef MSGPACK_PREPROCESSOR_CONTROL_EXPR_IF_HPP
# define MSGPACK_PREPROCESSOR_CONTROL_EXPR_IF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/expr_iif.hpp>
# include <rpc/msgpack/preprocessor/logical/bool.hpp>
#
# /* MSGPACK_PP_EXPR_IF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_EXPR_IF(cond, expr) MSGPACK_PP_EXPR_IIF(MSGPACK_PP_BOOL(cond), expr)
# else
#    define MSGPACK_PP_EXPR_IF(cond, expr) MSGPACK_PP_EXPR_IF_I(cond, expr)
#    define MSGPACK_PP_EXPR_IF_I(cond, expr) MSGPACK_PP_EXPR_IIF(MSGPACK_PP_BOOL(cond), expr)
# endif
#
# endif
