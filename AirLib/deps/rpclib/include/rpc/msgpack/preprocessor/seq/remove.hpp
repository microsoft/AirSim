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
# ifndef MSGPACK_PREPROCESSOR_SEQ_REMOVE_HPP
# define MSGPACK_PREPROCESSOR_SEQ_REMOVE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/seq/first_n.hpp>
# include <rpc/msgpack/preprocessor/seq/rest_n.hpp>
#
# /* MSGPACK_PP_SEQ_REMOVE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_REMOVE(seq, i) MSGPACK_PP_SEQ_FIRST_N(i, seq) MSGPACK_PP_SEQ_REST_N(MSGPACK_PP_INC(i), seq)
# else
#    define MSGPACK_PP_SEQ_REMOVE(seq, i) MSGPACK_PP_SEQ_REMOVE_I(seq, i)
#    define MSGPACK_PP_SEQ_REMOVE_I(seq, i) MSGPACK_PP_SEQ_FIRST_N(i, seq) MSGPACK_PP_SEQ_REST_N(MSGPACK_PP_INC(i), seq)
# endif
#
# endif
