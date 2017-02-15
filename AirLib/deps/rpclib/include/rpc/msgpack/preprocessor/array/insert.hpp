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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_INSERT_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_INSERT_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/array/elem.hpp>
# include <rpc/msgpack/preprocessor/array/push_back.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/comparison/not_equal.hpp>
# include <rpc/msgpack/preprocessor/control/deduce_d.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_INSERT */
#
# define MSGPACK_PP_ARRAY_INSERT(array, i, elem) MSGPACK_PP_ARRAY_INSERT_I(MSGPACK_PP_DEDUCE_D(), array, i, elem)
# define MSGPACK_PP_ARRAY_INSERT_I(d, array, i, elem) MSGPACK_PP_ARRAY_INSERT_D(d, array, i, elem)
#
# /* MSGPACK_PP_ARRAY_INSERT_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_INSERT_D(d, array, i, elem) MSGPACK_PP_TUPLE_ELEM(5, 3, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_INSERT_P, MSGPACK_PP_ARRAY_INSERT_O, (0, i, elem, (0, ()), array)))
# else
#    define MSGPACK_PP_ARRAY_INSERT_D(d, array, i, elem) MSGPACK_PP_ARRAY_INSERT_D_I(d, array, i, elem)
#    define MSGPACK_PP_ARRAY_INSERT_D_I(d, array, i, elem) MSGPACK_PP_TUPLE_ELEM(5, 3, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_INSERT_P, MSGPACK_PP_ARRAY_INSERT_O, (0, i, elem, (0, ()), array)))
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_ARRAY_INSERT_P(d, state) MSGPACK_PP_ARRAY_INSERT_P_I state
# else
#    define MSGPACK_PP_ARRAY_INSERT_P(d, state) MSGPACK_PP_ARRAY_INSERT_P_I(nil, nil, nil, MSGPACK_PP_TUPLE_ELEM(5, 3, state), MSGPACK_PP_TUPLE_ELEM(5, 4, state))
# endif
#
# define MSGPACK_PP_ARRAY_INSERT_P_I(_i, _ii, _iii, res, arr) MSGPACK_PP_NOT_EQUAL(MSGPACK_PP_ARRAY_SIZE(res), MSGPACK_PP_INC(MSGPACK_PP_ARRAY_SIZE(arr)))
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_ARRAY_INSERT_O(d, state) MSGPACK_PP_ARRAY_INSERT_O_I state
# else
#    define MSGPACK_PP_ARRAY_INSERT_O(d, state) MSGPACK_PP_ARRAY_INSERT_O_I(MSGPACK_PP_TUPLE_ELEM(5, 0, state), MSGPACK_PP_TUPLE_ELEM(5, 1, state), MSGPACK_PP_TUPLE_ELEM(5, 2, state), MSGPACK_PP_TUPLE_ELEM(5, 3, state), MSGPACK_PP_TUPLE_ELEM(5, 4, state))
# endif
#
# define MSGPACK_PP_ARRAY_INSERT_O_I(n, i, elem, res, arr) (MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(MSGPACK_PP_ARRAY_SIZE(res), i), MSGPACK_PP_INC(n), n), i, elem, MSGPACK_PP_ARRAY_PUSH_BACK(res, MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(MSGPACK_PP_ARRAY_SIZE(res), i), MSGPACK_PP_ARRAY_ELEM(n, arr), elem)), arr)
#
# endif
