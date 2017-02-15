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
# ifndef MSGPACK_PREPROCESSOR_SEQ_REVERSE_HPP
# define MSGPACK_PREPROCESSOR_SEQ_REVERSE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/facilities/empty.hpp>
# include <rpc/msgpack/preprocessor/seq/fold_left.hpp>
#
# /* MSGPACK_PP_SEQ_REVERSE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_REVERSE(seq) MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_REVERSE_O, MSGPACK_PP_EMPTY, seq)()
# else
#    define MSGPACK_PP_SEQ_REVERSE(seq) MSGPACK_PP_SEQ_REVERSE_I(seq)
#    define MSGPACK_PP_SEQ_REVERSE_I(seq) MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_REVERSE_O, MSGPACK_PP_EMPTY, seq)()
# endif
#
# define MSGPACK_PP_SEQ_REVERSE_O(s, state, elem) (elem) state
#
# /* MSGPACK_PP_SEQ_REVERSE_S */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_REVERSE_S(s, seq) MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_REVERSE_O, MSGPACK_PP_EMPTY, seq)()
# else
#    define MSGPACK_PP_SEQ_REVERSE_S(s, seq) MSGPACK_PP_SEQ_REVERSE_S_I(s, seq)
#    define MSGPACK_PP_SEQ_REVERSE_S_I(s, seq) MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_REVERSE_O, MSGPACK_PP_EMPTY, seq)()
# endif
#
# endif
