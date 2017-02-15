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
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_APPLY_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_APPLY_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/expr_iif.hpp>
# include <rpc/msgpack/preprocessor/detail/is_unary.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_APPLY */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_APPLY(x) MSGPACK_PP_APPLY_I(x)
#    define MSGPACK_PP_APPLY_I(x) MSGPACK_PP_EXPR_IIF(MSGPACK_PP_IS_UNARY(x), MSGPACK_PP_TUPLE_REM_1 x)
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_BCC()
#    define MSGPACK_PP_APPLY(x) MSGPACK_PP_APPLY_I(x)
#    define MSGPACK_PP_APPLY_I(x) MSGPACK_PP_APPLY_ ## x
#    define MSGPACK_PP_APPLY_(x) x
#    define MSGPACK_PP_APPLY_MSGPACK_PP_NIL
# else
#    define MSGPACK_PP_APPLY(x) MSGPACK_PP_EXPR_IIF(MSGPACK_PP_IS_UNARY(x), MSGPACK_PP_TUPLE_REM_1 x)
# endif
#
# endif
