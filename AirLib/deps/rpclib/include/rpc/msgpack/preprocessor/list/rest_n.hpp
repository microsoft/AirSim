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
# ifndef MSGPACK_PREPROCESSOR_LIST_REST_N_HPP
# define MSGPACK_PREPROCESSOR_LIST_REST_N_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_LIST_REST_N */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_REST_N(count, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_REST_N_P, MSGPACK_PP_LIST_REST_N_O, (list, count)))
# else
#    define MSGPACK_PP_LIST_REST_N(count, list) MSGPACK_PP_LIST_REST_N_I(count, list)
#    define MSGPACK_PP_LIST_REST_N_I(count, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_REST_N_P, MSGPACK_PP_LIST_REST_N_O, (list, count)))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_REST_N_P(d, lc) MSGPACK_PP_TUPLE_ELEM(2, 1, lc)
# else
#    define MSGPACK_PP_LIST_REST_N_P(d, lc) MSGPACK_PP_LIST_REST_N_P_I lc
#    define MSGPACK_PP_LIST_REST_N_P_I(list, count) count
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_REST_N_O(d, lc) (MSGPACK_PP_LIST_REST(MSGPACK_PP_TUPLE_ELEM(2, 0, lc)), MSGPACK_PP_DEC(MSGPACK_PP_TUPLE_ELEM(2, 1, lc)))
# else
#    define MSGPACK_PP_LIST_REST_N_O(d, lc) MSGPACK_PP_LIST_REST_N_O_I lc
#    define MSGPACK_PP_LIST_REST_N_O_I(list, count) (MSGPACK_PP_LIST_REST(list), MSGPACK_PP_DEC(count))
# endif
#
# /* MSGPACK_PP_LIST_REST_N_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_REST_N_D(d, count, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_REST_N_P, MSGPACK_PP_LIST_REST_N_O, (list, count)))
# else
#    define MSGPACK_PP_LIST_REST_N_D(d, count, list) MSGPACK_PP_LIST_REST_N_D_I(d, count, list)
#    define MSGPACK_PP_LIST_REST_N_D_I(d, count, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_REST_N_P, MSGPACK_PP_LIST_REST_N_O, (list, count)))
# endif
#
# endif
