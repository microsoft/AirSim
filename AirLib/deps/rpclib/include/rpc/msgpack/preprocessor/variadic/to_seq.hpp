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
# ifndef MSGPACK_PREPROCESSOR_VARIADIC_TO_SEQ_HPP
# define MSGPACK_PREPROCESSOR_VARIADIC_TO_SEQ_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/to_seq.hpp>
#
# /* MSGPACK_PP_VARIADIC_TO_SEQ */
#
# if MSGPACK_PP_VARIADICS
#    define MSGPACK_PP_VARIADIC_TO_SEQ(...) MSGPACK_PP_TUPLE_TO_SEQ((__VA_ARGS__))
# endif
#
# endif
