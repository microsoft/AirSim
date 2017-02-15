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
# ifndef MSGPACK_PREPROCESSOR_SEQ_TRANSFORM_HPP
# define MSGPACK_PREPROCESSOR_SEQ_TRANSFORM_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/seq/fold_left.hpp>
# include <rpc/msgpack/preprocessor/seq/seq.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_SEQ_TRANSFORM */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_TRANSFORM(op, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_TRANSFORM_O, (op, data, (nil)), seq)))
# else
#    define MSGPACK_PP_SEQ_TRANSFORM(op, data, seq) MSGPACK_PP_SEQ_TRANSFORM_I(op, data, seq)
#    define MSGPACK_PP_SEQ_TRANSFORM_I(op, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_TRANSFORM_O, (op, data, (nil)), seq)))
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_SEQ_TRANSFORM_O(s, state, elem) MSGPACK_PP_SEQ_TRANSFORM_O_IM(s, MSGPACK_PP_TUPLE_REM_3 state, elem)
#    define MSGPACK_PP_SEQ_TRANSFORM_O_IM(s, im, elem) MSGPACK_PP_SEQ_TRANSFORM_O_I(s, im, elem)
# else
#    define MSGPACK_PP_SEQ_TRANSFORM_O(s, state, elem) MSGPACK_PP_SEQ_TRANSFORM_O_I(s, MSGPACK_PP_TUPLE_ELEM(3, 0, state), MSGPACK_PP_TUPLE_ELEM(3, 1, state), MSGPACK_PP_TUPLE_ELEM(3, 2, state), elem)
# endif
#
# define MSGPACK_PP_SEQ_TRANSFORM_O_I(s, op, data, res, elem) (op, data, res (op(s, data, elem)))
#
# /* MSGPACK_PP_SEQ_TRANSFORM_S */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_TRANSFORM_S(s, op, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_TRANSFORM_O, (op, data, (nil)), seq)))
# else
#    define MSGPACK_PP_SEQ_TRANSFORM_S(s, op, data, seq) MSGPACK_PP_SEQ_TRANSFORM_S_I(s, op, data, seq)
#    define MSGPACK_PP_SEQ_TRANSFORM_S_I(s, op, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_TRANSFORM_O, (op, data, (nil)), seq)))
# endif
#
# endif
