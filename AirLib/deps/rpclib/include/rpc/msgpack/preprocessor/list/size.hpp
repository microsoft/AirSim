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
# ifndef MSGPACK_PREPROCESSOR_LIST_SIZE_HPP
# define MSGPACK_PREPROCESSOR_LIST_SIZE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_SIZE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_SIZE(list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_SIZE_P, MSGPACK_PP_LIST_SIZE_O, (0, list)))
# else
#    define MSGPACK_PP_LIST_SIZE(list) MSGPACK_PP_LIST_SIZE_I(list)
#    define MSGPACK_PP_LIST_SIZE_I(list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_LIST_SIZE_P, MSGPACK_PP_LIST_SIZE_O, (0, list)))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_SIZE_P(d, rl) MSGPACK_PP_LIST_IS_CONS(MSGPACK_PP_TUPLE_ELEM(2, 1, rl))
# else
#    define MSGPACK_PP_LIST_SIZE_P(d, rl) MSGPACK_PP_LIST_SIZE_P_I(MSGPACK_PP_TUPLE_REM_2 rl)
#    define MSGPACK_PP_LIST_SIZE_P_I(im) MSGPACK_PP_LIST_SIZE_P_II(im)
#    define MSGPACK_PP_LIST_SIZE_P_II(r, l) MSGPACK_PP_LIST_IS_CONS(l)
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_SIZE_O(d, rl) (MSGPACK_PP_INC(MSGPACK_PP_TUPLE_ELEM(2, 0, rl)), MSGPACK_PP_LIST_REST(MSGPACK_PP_TUPLE_ELEM(2, 1, rl)))
# else
#    define MSGPACK_PP_LIST_SIZE_O(d, rl) MSGPACK_PP_LIST_SIZE_O_I(MSGPACK_PP_TUPLE_REM_2 rl)
#    define MSGPACK_PP_LIST_SIZE_O_I(im) MSGPACK_PP_LIST_SIZE_O_II(im)
#    define MSGPACK_PP_LIST_SIZE_O_II(r, l) (MSGPACK_PP_INC(r), MSGPACK_PP_LIST_REST(l))
# endif
#
# /* MSGPACK_PP_LIST_SIZE_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_SIZE_D(d, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_SIZE_P, MSGPACK_PP_LIST_SIZE_O, (0, list)))
# else
#    define MSGPACK_PP_LIST_SIZE_D(d, list) MSGPACK_PP_LIST_SIZE_D_I(d, list)
#    define MSGPACK_PP_LIST_SIZE_D_I(d, list) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_LIST_SIZE_P, MSGPACK_PP_LIST_SIZE_O, (0, list)))
# endif
#
# endif
