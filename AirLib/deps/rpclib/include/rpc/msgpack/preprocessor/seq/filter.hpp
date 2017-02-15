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
# ifndef MSGPACK_PREPROCESSOR_SEQ_FILTER_HPP
# define MSGPACK_PREPROCESSOR_SEQ_FILTER_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/expr_if.hpp>
# include <rpc/msgpack/preprocessor/facilities/empty.hpp>
# include <rpc/msgpack/preprocessor/seq/fold_left.hpp>
# include <rpc/msgpack/preprocessor/seq/seq.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_SEQ_FILTER */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_FILTER(pred, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_FILTER_O, (pred, data, (nil)), seq)))
# else
#    define MSGPACK_PP_SEQ_FILTER(pred, data, seq) MSGPACK_PP_SEQ_FILTER_I(pred, data, seq)
#    define MSGPACK_PP_SEQ_FILTER_I(pred, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT(MSGPACK_PP_SEQ_FILTER_O, (pred, data, (nil)), seq)))
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_SEQ_FILTER_O(s, st, elem) MSGPACK_PP_SEQ_FILTER_O_IM(s, MSGPACK_PP_TUPLE_REM_3 st, elem)
#    define MSGPACK_PP_SEQ_FILTER_O_IM(s, im, elem) MSGPACK_PP_SEQ_FILTER_O_I(s, im, elem)
# else
#    define MSGPACK_PP_SEQ_FILTER_O(s, st, elem) MSGPACK_PP_SEQ_FILTER_O_I(s, MSGPACK_PP_TUPLE_ELEM(3, 0, st), MSGPACK_PP_TUPLE_ELEM(3, 1, st), MSGPACK_PP_TUPLE_ELEM(3, 2, st), elem)
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_DMC()
#   define MSGPACK_PP_SEQ_FILTER_O_I(s, pred, data, res, elem) (pred, data, res MSGPACK_PP_EXPR_IF(pred(s, data, elem), (elem)))
# else
#   define MSGPACK_PP_SEQ_FILTER_O_I(s, pred, data, res, elem) (pred, data, res MSGPACK_PP_EXPR_IF(pred##(s, data, elem), (elem)))
# endif
#
# /* MSGPACK_PP_SEQ_FILTER_S */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_FILTER_S(s, pred, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_FILTER_O, (pred, data, (nil)), seq)))
# else
#    define MSGPACK_PP_SEQ_FILTER_S(s, pred, data, seq) MSGPACK_PP_SEQ_FILTER_S_I(s, pred, data, seq)
#    define MSGPACK_PP_SEQ_FILTER_S_I(s, pred, data, seq) MSGPACK_PP_SEQ_TAIL(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_SEQ_FOLD_LEFT_ ## s(MSGPACK_PP_SEQ_FILTER_O, (pred, data, (nil)), seq)))
# endif
#
# endif
