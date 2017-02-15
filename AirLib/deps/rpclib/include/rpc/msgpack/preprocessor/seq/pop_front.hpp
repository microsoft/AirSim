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
# ifndef MSGPACK_PREPROCESSOR_SEQ_POP_FRONT_HPP
# define MSGPACK_PREPROCESSOR_SEQ_POP_FRONT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/seq/seq.hpp>
#
# /* MSGPACK_PP_SEQ_POP_FRONT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_POP_FRONT(seq) MSGPACK_PP_SEQ_TAIL(seq)
# else
#    define MSGPACK_PP_SEQ_POP_FRONT(seq) MSGPACK_PP_SEQ_POP_FRONT_I(seq)
#    define MSGPACK_PP_SEQ_POP_FRONT_I(seq) MSGPACK_PP_SEQ_TAIL(seq)
# endif
#
# endif
