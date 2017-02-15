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
# ifndef MSGPACK_PREPROCESSOR_ARITHMETIC_MUL_HPP
# define MSGPACK_PREPROCESSOR_ARITHMETIC_MUL_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/add.hpp>
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_MUL */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_MUL(x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_WHILE(MSGPACK_PP_MUL_P, MSGPACK_PP_MUL_O, (0, x, y)))
# else
#    define MSGPACK_PP_MUL(x, y) MSGPACK_PP_MUL_I(x, y)
#    define MSGPACK_PP_MUL_I(x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_WHILE(MSGPACK_PP_MUL_P, MSGPACK_PP_MUL_O, (0, x, y)))
# endif
#
# define MSGPACK_PP_MUL_P(d, rxy) MSGPACK_PP_TUPLE_ELEM(3, 2, rxy)
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_MUL_O(d, rxy) MSGPACK_PP_MUL_O_IM(d, MSGPACK_PP_TUPLE_REM_3 rxy)
#    define MSGPACK_PP_MUL_O_IM(d, im) MSGPACK_PP_MUL_O_I(d, im)
# else
#    define MSGPACK_PP_MUL_O(d, rxy) MSGPACK_PP_MUL_O_I(d, MSGPACK_PP_TUPLE_ELEM(3, 0, rxy), MSGPACK_PP_TUPLE_ELEM(3, 1, rxy), MSGPACK_PP_TUPLE_ELEM(3, 2, rxy))
# endif
#
# define MSGPACK_PP_MUL_O_I(d, r, x, y) (MSGPACK_PP_ADD_D(d, r, x), x, MSGPACK_PP_DEC(y))
#
# /* MSGPACK_PP_MUL_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_MUL_D(d, x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_MUL_P, MSGPACK_PP_MUL_O, (0, x, y)))
# else
#    define MSGPACK_PP_MUL_D(d, x, y) MSGPACK_PP_MUL_D_I(d, x, y)
#    define MSGPACK_PP_MUL_D_I(d, x, y) MSGPACK_PP_TUPLE_ELEM(3, 0, MSGPACK_PP_WHILE_ ## d(MSGPACK_PP_MUL_P, MSGPACK_PP_MUL_O, (0, x, y)))
# endif
#
# endif
