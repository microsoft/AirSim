# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_CONTROL_EXPR_IIF_HPP
# define MSGPACK_PREPROCESSOR_CONTROL_EXPR_IIF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_EXPR_IIF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_EXPR_IIF(bit, expr) MSGPACK_PP_EXPR_IIF_I(bit, expr)
# else
#    define MSGPACK_PP_EXPR_IIF(bit, expr) MSGPACK_PP_EXPR_IIF_OO((bit, expr))
#    define MSGPACK_PP_EXPR_IIF_OO(par) MSGPACK_PP_EXPR_IIF_I ## par
# endif
#
# define MSGPACK_PP_EXPR_IIF_I(bit, expr) MSGPACK_PP_EXPR_IIF_ ## bit(expr)
#
# define MSGPACK_PP_EXPR_IIF_0(expr)
# define MSGPACK_PP_EXPR_IIF_1(expr) expr
#
# endif
