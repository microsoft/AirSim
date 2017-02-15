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
# ifndef MSGPACK_PREPROCESSOR_LIST_FIRST_N_HPP
# define MSGPACK_PREPROCESSOR_LIST_FIRST_N_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/list/reverse.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_FIRST_N */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FIRST_N(count, list) MSGPACK_PP_LIST_REVERSE(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_FIRST_N_P, MSGPACK_PP_LIST_FIRST_N_O, (count, list, MSGPACK_PP_NIL))))
# else
#    define MSGPACK_PP_LIST_FIRST_N(count, list) MSGPACK_PP_LIST_FIRST_N_I(count, list)
#    define MSGPACK_PP_LIST_FIRST_N_I(count, list) MSGPACK_PP_LIST_REVERSE(MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_FIRST_N_P, MSGPACK_PP_LIST_FIRST_N_O, (count, list, MSGPACK_PP_NIL))))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FIRST_N_P(d, data) MSGPACK_PP_TUPLE_ELEM(3, 0, data)
# else
#    define MSGPACK_PP_LIST_FIRST_N_P(d, data) MSGPACK_PP_LIST_FIRST_N_P_I data
#    define MSGPACK_PP_LIST_FIRST_N_P_I(c, l, nl) c
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_LIST_FIRST_N_O(d, data) MSGPACK_PP_LIST_FIRST_N_O_D data
# else
#    define MSGPACK_PP_LIST_FIRST_N_O(d, data) MSGPACK_PP_LIST_FIRST_N_O_D(MSGPACK_PP_TUPLE_ELEM(3, 0, data), MSGPACK_PP_TUPLE_ELEM(3, 1, data), MSGPACK_PP_TUPLE_ELEM(3, 2, data))
# endif
#
# define MSGPACK_PP_LIST_FIRST_N_O_D(c, l, nl) (MSGPACK_PP_DEC(c), MSGPACK_PP_LIST_REST(l), (MSGPACK_PP_LIST_FIRST(l), nl))
#
# /* MSGPACK_PP_LIST_FIRST_N_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FIRST_N_D(d, count, list) MSGPACK_PP_LIST_REVERSE_D(d, MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_FIRST_N_P, MSGPACK_PP_LIST_FIRST_N_O, (count, list, MSGPACK_PP_NIL))))
# else
#    define MSGPACK_PP_LIST_FIRST_N_D(d, count, list) MSGPACK_PP_LIST_FIRST_N_D_I(d, count, list)
#    define MSGPACK_PP_LIST_FIRST_N_D_I(d, count, list) MSGPACK_PP_LIST_REVERSE_D(d, MSGPACK_PP_TUPLE_ELEM(3, 2, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_FIRST_N_P, MSGPACK_PP_LIST_FIRST_N_O, (count, list, MSGPACK_PP_NIL))))
# endif
#
# endif
