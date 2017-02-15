# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2012.                                  *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_SEQ_VARIADIC_SEQ_TO_SEQ_HPP
# define MSGPACK_PREPROCESSOR_SEQ_VARIADIC_SEQ_TO_SEQ_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_VARIADIC_SEQ_TO_SEQ */
#
# if MSGPACK_PP_VARIADICS
#    define MSGPACK_PP_VARIADIC_SEQ_TO_SEQ(vseq) MSGPACK_PP_CAT(MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_A vseq, 0)
#    define MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_A(...) ((__VA_ARGS__)) MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_B
#    define MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_B(...) ((__VA_ARGS__)) MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_A
#    define MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_A0
#    define MSGPACK_PP_VARIADIC_SEQ_TO_SEQ_B0
# endif
#
# endif
