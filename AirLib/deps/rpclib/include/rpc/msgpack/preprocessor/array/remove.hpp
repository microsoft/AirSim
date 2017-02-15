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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_REMOVE_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_REMOVE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/array/elem.hpp>
# include <rpc/msgpack/preprocessor/array/push_back.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/comparison/not_equal.hpp>
# include <rpc/msgpack/preprocessor/control/deduce_d.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/eat.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_REMOVE */
#
# define MSGPACK_PP_ARRAY_REMOVE(array, i) MSGPACK_PP_ARRAY_REMOVE_I(MSGPACK_PP_DEDUCE_D(), array, i)
# define MSGPACK_PP_ARRAY_REMOVE_I(d, array, i) MSGPACK_PP_ARRAY_REMOVE_D(d, array, i)
#
# /* MSGPACK_PP_ARRAY_REMOVE_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_REMOVE_D(d, array, i) MSGPACK_PP_TUPLE_ELEM(4, 2, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_REMOVE_P, MSGPACK_PP_ARRAY_REMOVE_O, (0, i, (0, ()), array)))
# else
#    define MSGPACK_PP_ARRAY_REMOVE_D(d, array, i) MSGPACK_PP_ARRAY_REMOVE_D_I(d, array, i)
#    define MSGPACK_PP_ARRAY_REMOVE_D_I(d, array, i) MSGPACK_PP_TUPLE_ELEM(4, 2, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ARRAY_REMOVE_P, MSGPACK_PP_ARRAY_REMOVE_O, (0, i, (0, ()), array)))
# endif
#
# define MSGPACK_PP_ARRAY_REMOVE_P(d, st) MSGPACK_PP_NOT_EQUAL(MSGPACK_PP_TUPLE_ELEM(4, 0, st), MSGPACK_PP_ARRAY_SIZE(MSGPACK_PP_TUPLE_ELEM(4, 3, st)))
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_ARRAY_REMOVE_O(d, st) MSGPACK_PP_ARRAY_REMOVE_O_I st
# else
#    define MSGPACK_PP_ARRAY_REMOVE_O(d, st) MSGPACK_PP_ARRAY_REMOVE_O_I(MSGPACK_PP_TUPLE_ELEM(4, 0, st), MSGPACK_PP_TUPLE_ELEM(4, 1, st), MSGPACK_PP_TUPLE_ELEM(4, 2, st), MSGPACK_PP_TUPLE_ELEM(4, 3, st))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_DMC()
#    define MSGPACK_PP_ARRAY_REMOVE_O_I(n, i, res, arr) (MSGPACK_PP_INC(n), i, MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(n, i), MSGPACK_PP_ARRAY_PUSH_BACK, res MSGPACK_PP_TUPLE_EAT_2)(res, MSGPACK_PP_ARRAY_ELEM(n, arr)), arr)
# else
#    define MSGPACK_PP_ARRAY_REMOVE_O_I(n, i, res, arr) (MSGPACK_PP_INC(n), i, MSGPACK_PP_IIF(MSGPACK_PP_NOT_EQUAL(n, i), MSGPACK_PP_ARRAY_PUSH_BACK, MSGPACK_PP_TUPLE_ELEM_2_0)(res, MSGPACK_PP_ARRAY_ELEM(n, arr)), arr)
# endif
#
# endif
