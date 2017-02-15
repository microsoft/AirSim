# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_LIST_FILTER_HPP
# define MSGPACK_PREPROCESSOR_LIST_FILTER_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
# include <rpc/msgpack/preprocessor/list/fold_right.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_FILTER */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FILTER(pred, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_FILTER_O, (pred, data, MSGPACK_PP_NIL), list))
# else
#    define MSGPACK_PP_LIST_FILTER(pred, data, list) MSGPACK_PP_LIST_FILTER_I(pred, data, list)
#    define MSGPACK_PP_LIST_FILTER_I(pred, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_FILTER_O, (pred, data, MSGPACK_PP_NIL), list))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FILTER_O(d, pdr, elem) MSGPACK_PP_LIST_FILTER_O_D(d, MSGPACK_PP_TUPLE_ELEM(3, 0, pdr), MSGPACK_PP_TUPLE_ELEM(3, 1, pdr), MSGPACK_PP_TUPLE_ELEM(3, 2, pdr), elem)
# else
#    define MSGPACK_PP_LIST_FILTER_O(d, pdr, elem) MSGPACK_PP_LIST_FILTER_O_I(d, MSGPACK_PP_TUPLE_REM_3 pdr, elem)
#    define MSGPACK_PP_LIST_FILTER_O_I(d, im, elem) MSGPACK_PP_LIST_FILTER_O_D(d, im, elem)
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_DMC()
#    define MSGPACK_PP_LIST_FILTER_O_D(d, pred, data, res, elem) (pred, data, MSGPACK_PP_IF(pred(d, data, elem), (elem, res), res))
# else
#    define MSGPACK_PP_LIST_FILTER_O_D(d, pred, data, res, elem) (pred, data, MSGPACK_PP_IF(pred##(d, data, elem), (elem, res), res))
# endif
#
# /* MSGPACK_PP_LIST_FILTER_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FILTER_D(d, pred, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_FILTER_O, (pred, data, MSGPACK_PP_NIL), list))
# else
#    define MSGPACK_PP_LIST_FILTER_D(d, pred, data, list) MSGPACK_PP_LIST_FILTER_D_I(d, pred, data, list)
#    define MSGPACK_PP_LIST_FILTER_D_I(d, pred, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_FILTER_O, (pred, data, MSGPACK_PP_NIL), list))
# endif
#
# endif
