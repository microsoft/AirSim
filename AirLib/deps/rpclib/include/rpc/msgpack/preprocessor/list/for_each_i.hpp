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
# ifndef MSGPACK_PREPROCESSOR_LIST_LIST_FOR_EACH_I_HPP
# define MSGPACK_PREPROCESSOR_LIST_LIST_FOR_EACH_I_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/repetition/for.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_FOR_EACH_I */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG() && ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_LIST_FOR_EACH_I(macro, data, list) MSGPACK_PP_FOR((macro, data, list, 0), MSGPACK_PP_LIST_FOR_EACH_I_P, MSGPACK_PP_LIST_FOR_EACH_I_O, MSGPACK_PP_LIST_FOR_EACH_I_M)
# else
#    define MSGPACK_PP_LIST_FOR_EACH_I(macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I_I(macro, data, list)
#    define MSGPACK_PP_LIST_FOR_EACH_I_I(macro, data, list) MSGPACK_PP_FOR((macro, data, list, 0), MSGPACK_PP_LIST_FOR_EACH_I_P, MSGPACK_PP_LIST_FOR_EACH_I_O, MSGPACK_PP_LIST_FOR_EACH_I_M)
# endif
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_LIST_FOR_EACH_I_P(r, x) MSGPACK_PP_LIST_FOR_EACH_I_P_D x
#    define MSGPACK_PP_LIST_FOR_EACH_I_P_D(m, d, l, i) MSGPACK_PP_LIST_IS_CONS(l)
# else
#    define MSGPACK_PP_LIST_FOR_EACH_I_P(r, x) MSGPACK_PP_LIST_IS_CONS(MSGPACK_PP_TUPLE_ELEM(4, 2, x))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_LIST_FOR_EACH_I_O(r, x) MSGPACK_PP_LIST_FOR_EACH_I_O_D x
#    define MSGPACK_PP_LIST_FOR_EACH_I_O_D(m, d, l, i) (m, d, MSGPACK_PP_LIST_REST(l), MSGPACK_PP_INC(i))
# else
#    define MSGPACK_PP_LIST_FOR_EACH_I_O(r, x) (MSGPACK_PP_TUPLE_ELEM(4, 0, x), MSGPACK_PP_TUPLE_ELEM(4, 1, x), MSGPACK_PP_LIST_REST(MSGPACK_PP_TUPLE_ELEM(4, 2, x)), MSGPACK_PP_INC(MSGPACK_PP_TUPLE_ELEM(4, 3, x)))
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FOR_EACH_I_M(r, x) MSGPACK_PP_LIST_FOR_EACH_I_M_D(r, MSGPACK_PP_TUPLE_ELEM(4, 0, x), MSGPACK_PP_TUPLE_ELEM(4, 1, x), MSGPACK_PP_TUPLE_ELEM(4, 2, x), MSGPACK_PP_TUPLE_ELEM(4, 3, x))
# else
#    define MSGPACK_PP_LIST_FOR_EACH_I_M(r, x) MSGPACK_PP_LIST_FOR_EACH_I_M_I(r, MSGPACK_PP_TUPLE_REM_4 x)
#    define MSGPACK_PP_LIST_FOR_EACH_I_M_I(r, x_e) MSGPACK_PP_LIST_FOR_EACH_I_M_D(r, x_e)
# endif
#
# define MSGPACK_PP_LIST_FOR_EACH_I_M_D(r, m, d, l, i) m(r, d, i, MSGPACK_PP_LIST_FIRST(l))
#
# /* MSGPACK_PP_LIST_FOR_EACH_I_R */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FOR_EACH_I_R(r, macro, data, list) MSGPACK_PP_FOR_ ## r((macro, data, list, 0), MSGPACK_PP_LIST_FOR_EACH_I_P, MSGPACK_PP_LIST_FOR_EACH_I_O, MSGPACK_PP_LIST_FOR_EACH_I_M)
# else
#    define MSGPACK_PP_LIST_FOR_EACH_I_R(r, macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I_R_I(r, macro, data, list)
#    define MSGPACK_PP_LIST_FOR_EACH_I_R_I(r, macro, data, list) MSGPACK_PP_FOR_ ## r((macro, data, list, 0), MSGPACK_PP_LIST_FOR_EACH_I_P, MSGPACK_PP_LIST_FOR_EACH_I_O, MSGPACK_PP_LIST_FOR_EACH_I_M)
# endif
#
# endif
