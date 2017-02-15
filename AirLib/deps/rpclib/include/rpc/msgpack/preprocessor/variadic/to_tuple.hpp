# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2011.                                    *
#  *     (C) Copyright Paul Mensonides 2011.                                  *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_VARIADIC_TO_TUPLE_HPP
# define MSGPACK_PREPROCESSOR_VARIADIC_TO_TUPLE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_VARIADIC_TO_TUPLE */
#
# if MSGPACK_PP_VARIADICS
#    define MSGPACK_PP_VARIADIC_TO_TUPLE(...) (__VA_ARGS__)
# endif
#
# endif
