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
# ifndef MSGPACK_PREPROCESSOR_ARITHMETIC_ADD_HPP
# define MSGPACK_PREPROCESSOR_ARITHMETIC_ADD_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ADD */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ADD(x, y) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_ADD_P, MSGPACK_PP_ADD_O, (x, y)))
# else
#    define MSGPACK_PP_ADD(x, y) MSGPACK_PP_ADD_I(x, y)
#    define MSGPACK_PP_ADD_I(x, y) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE(MSGPACK_PP_ADD_P, MSGPACK_PP_ADD_O, (x, y)))
# endif
#
# define MSGPACK_PP_ADD_P(d, xy) MSGPACK_PP_TUPLE_ELEM(2, 1, xy)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_ADD_O(d, xy) MSGPACK_PP_ADD_O_I xy
# else
#    define MSGPACK_PP_ADD_O(d, xy) MSGPACK_PP_ADD_O_I(MSGPACK_PP_TUPLE_ELEM(2, 0, xy), MSGPACK_PP_TUPLE_ELEM(2, 1, xy))
# endif
#
# define MSGPACK_PP_ADD_O_I(x, y) (MSGPACK_PP_INC(x), MSGPACK_PP_DEC(y))
#
# /* MSGPACK_PP_ADD_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ADD_D(d, x, y) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ADD_P, MSGPACK_PP_ADD_O, (x, y)))
# else
#    define MSGPACK_PP_ADD_D(d, x, y) MSGPACK_PP_ADD_D_I(d, x, y)
#    define MSGPACK_PP_ADD_D_I(d, x, y) MSGPACK_PP_TUPLE_ELEM(2, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_ADD_P, MSGPACK_PP_ADD_O, (x, y)))
# endif
#
# endif
