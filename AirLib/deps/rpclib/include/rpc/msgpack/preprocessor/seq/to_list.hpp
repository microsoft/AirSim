# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2011.                                  *
#  *     (C) Copyright Edward Diener 2011.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_SEQ_TO_LIST_HPP
# define MSGPACK_PREPROCESSOR_SEQ_TO_LIST_HPP
#
# include <rpc/msgpack/preprocessor/punctuation/comma.hpp>
# include <rpc/msgpack/preprocessor/punctuation/paren.hpp>
# include <rpc/msgpack/preprocessor/seq/detail/binary_transform.hpp>
#
# /* MSGPACK_PP_SEQ_TO_LIST */
#
# define MSGPACK_PP_SEQ_TO_LIST(seq) MSGPACK_PP_SEQ_TO_LIST_I(MSGPACK_PP_SEQ_BINARY_TRANSFORM(seq))
# define MSGPACK_PP_SEQ_TO_LIST_I(bseq) MSGPACK_PP_SEQ_TO_LIST_A bseq MSGPACK_PP_NIL MSGPACK_PP_SEQ_TO_LIST_B bseq
# define MSGPACK_PP_SEQ_TO_LIST_A(m, e) m(MSGPACK_PP_LPAREN() e MSGPACK_PP_COMMA() MSGPACK_PP_SEQ_TO_LIST_A_ID)
# define MSGPACK_PP_SEQ_TO_LIST_A_ID() MSGPACK_PP_SEQ_TO_LIST_A
# define MSGPACK_PP_SEQ_TO_LIST_B(m, e) m(MSGPACK_PP_RPAREN() MSGPACK_PP_SEQ_TO_LIST_B_ID)
# define MSGPACK_PP_SEQ_TO_LIST_B_ID() MSGPACK_PP_SEQ_TO_LIST_B
#
# endif
