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
# ifndef MSGPACK_PREPROCESSOR_DETAIL_IS_UNARY_HPP
# define MSGPACK_PREPROCESSOR_DETAIL_IS_UNARY_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/detail/check.hpp>
#
# /* MSGPACK_PP_IS_UNARY */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_IS_UNARY(x) MSGPACK_PP_CHECK(x, MSGPACK_PP_IS_UNARY_CHECK)
# else
#    define MSGPACK_PP_IS_UNARY(x) MSGPACK_PP_IS_UNARY_I(x)
#    define MSGPACK_PP_IS_UNARY_I(x) MSGPACK_PP_CHECK(x, MSGPACK_PP_IS_UNARY_CHECK)
# endif
#
# define MSGPACK_PP_IS_UNARY_CHECK(a) 1
# define MSGPACK_PP_CHECK_RESULT_MSGPACK_PP_IS_UNARY_CHECK 0, MSGPACK_PP_NIL
#
# endif
