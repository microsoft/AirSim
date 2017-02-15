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
# ifndef MSGPACK_PREPROCESSOR_SEQ_SEQ_HPP
# define MSGPACK_PREPROCESSOR_SEQ_SEQ_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/seq/elem.hpp>
#
# /* MSGPACK_PP_SEQ_HEAD */
#
# define MSGPACK_PP_SEQ_HEAD(seq) MSGPACK_PP_SEQ_ELEM(0, seq)
#
# /* MSGPACK_PP_SEQ_TAIL */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_SEQ_TAIL(seq) MSGPACK_PP_SEQ_TAIL_1((seq))
#    define MSGPACK_PP_SEQ_TAIL_1(par) MSGPACK_PP_SEQ_TAIL_2 ## par
#    define MSGPACK_PP_SEQ_TAIL_2(seq) MSGPACK_PP_SEQ_TAIL_I ## seq
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_SEQ_TAIL(seq) MSGPACK_PP_SEQ_TAIL_ID(MSGPACK_PP_SEQ_TAIL_I seq)
#    define MSGPACK_PP_SEQ_TAIL_ID(id) id
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_TAIL(seq) MSGPACK_PP_SEQ_TAIL_D(seq)
#    define MSGPACK_PP_SEQ_TAIL_D(seq) MSGPACK_PP_SEQ_TAIL_I seq
# else
#    define MSGPACK_PP_SEQ_TAIL(seq) MSGPACK_PP_SEQ_TAIL_I seq
# endif
#
# define MSGPACK_PP_SEQ_TAIL_I(x)
#
# /* MSGPACK_PP_SEQ_NIL */
#
# define MSGPACK_PP_SEQ_NIL(x) (x)
#
# endif
