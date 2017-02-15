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
# ifndef MSGPACK_PREPROCESSOR_SEQ_TO_TUPLE_HPP
# define MSGPACK_PREPROCESSOR_SEQ_TO_TUPLE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/seq/enum.hpp>
#
# /* MSGPACK_PP_SEQ_TO_TUPLE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_TO_TUPLE(seq) (MSGPACK_PP_SEQ_ENUM(seq))
# else
#    define MSGPACK_PP_SEQ_TO_TUPLE(seq) MSGPACK_PP_SEQ_TO_TUPLE_I(seq)
#    define MSGPACK_PP_SEQ_TO_TUPLE_I(seq) (MSGPACK_PP_SEQ_ENUM(seq))
# endif
#
# endif
