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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_REPLACE_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_REPLACE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/array/elem.hpp>
# include <rpc/msgpack/preprocessor/array/push_back.hpp>
# include <rpc/msgpack/preprocessor/comparison/not_equal.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/deduce_d.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_REPLACE */
#
# define MSGPACK_PP_ARRAY_REPLACE(array, i, elem) MSGPACK_PP_ARRAY_REPLACE_I(MSGPACK_PP_DEDUCE_D(), array, i, elem)
# define MSGPACK_PP_ARRAY_REPLACE_I(d, array, i, elem) MSGPACK_PP_ARRAY_REPLACE_D(d, array, i, elem)
#
# /* MSGPACK_PP_ARRAY_REPLACE_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_REPLACE_D(d, array, i, elem) MSGPACK_PP_TUPLE_ELEM(5, 3, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_REPLACE_P, MSGPACK_PP_ARRAY_REPLACE_O, (0, i, elem, (0, ()), array)))
# else
#    define MSGPACK_PP_ARRAY_REPLACE_D(d, array, i, elem) MSGPACK_PP_ARRAY_REPLACE_D_I(d, array, i, elem)
#    define MSGPACK_PP_ARRAY_REPLACE_D_I(d, array, i, elem) MSGPACK_PP_TUPLE_ELEM(5, 3, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_REPLACE_P, MSGPACK_PP_ARRAY_REPLACE_O, (0, i, elem, (0, ()), array)))
# endif
#
# define MSGPACK_PP_ARRAY_REPLACE_P(d, state) MSGPACK_PP_NOT_EQUAL(MSGPACK_PP_TUPLE_ELEM(5, 0, state), MSGPACK_PP_ARRAY_SIZE(MSGPACK_PP_TUPLE_ELEM(5, 4, state)))
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_ARRAY_REPLACE_O(d, state) MSGPACK_PP_ARRAY_REPLACE_O_I state
# else
#    define MSGPACK_PP_ARRAY_REPLACE_O(d, state) MSGPACK_PP_ARRAY_REPLACE_O_I(MSGPACK_PP_TUPLE_ELEM(5, 0, state), MSGPACK_PP_TUPLE_ELEM(5, 1, state), MSGPACK_PP_TUPLE_ELEM(5, 2, state), MSGPACK_PP_TUPLE_ELEM(5, 3, state), MSGPACK_PP_TUPLE_ELEM(5, 4, state))
# endif
#
# define MSGPACK_PP_ARRAY_REPLACE_O_I(n, i, elem, res, arr) (MSGPACK_PP_INC(n), i, elem, MSGPACK_PP_ARRAY_PUSH_BACK(res, MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(n, i), MSGPACK_PP_ARRAY_ELEM(n, arr), elem)), arr)
#
# endif
