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
# ifndef MSGPACK_PREPROCESSOR_LIST_TRANSFORM_HPP
# define MSGPACK_PREPROCESSOR_LIST_TRANSFORM_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/fold_right.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_TRANSFORM */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_TRANSFORM(op, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_TRANSFORM_O, (op, data, MSGPACK_PP_NIL), list))
# else
#    define MSGPACK_PP_LIST_TRANSFORM(op, data, list) MSGPACK_PP_LIST_TRANSFORM_I(op, data, list)
#    define MSGPACK_PP_LIST_TRANSFORM_I(op, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_TRANSFORM_O, (op, data, MSGPACK_PP_NIL), list))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_TRANSFORM_O(d, odr, elem) MSGPACK_PP_LIST_TRANSFORM_O_D(d, MSGPACK_PP_TUPLE_ELEM(3, 0, odr), MSGPACK_PP_TUPLE_ELEM(3, 1, odr), MSGPACK_PP_TUPLE_ELEM(3, 2, odr), elem)
# else
#    define MSGPACK_PP_LIST_TRANSFORM_O(d, odr, elem) MSGPACK_PP_LIST_TRANSFORM_O_I(d, MSGPACK_PP_TUPLE_REM_3 odr, elem)
#    define MSGPACK_PP_LIST_TRANSFORM_O_I(d, im, elem) MSGPACK_PP_LIST_TRANSFORM_O_D(d, im, elem)
# endif
#
# define MSGPACK_PP_LIST_TRANSFORM_O_D(d, op, data, res, elem) (op, data, (op(d, data, elem), res))
#
# /* MSGPACK_PP_LIST_TRANSFORM_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_TRANSFORM_D(d, op, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_TRANSFORM_O, (op, data, MSGPACK_PP_NIL), list))
# else
#    define MSGPACK_PP_LIST_TRANSFORM_D(d, op, data, list) MSGPACK_PP_LIST_TRANSFORM_D_I(d, op, data, list)
#    define MSGPACK_PP_LIST_TRANSFORM_D_I(d, op, data, list) MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_TRANSFORM_O, (op, data, MSGPACK_PP_NIL), list))
# endif
#
# endif
