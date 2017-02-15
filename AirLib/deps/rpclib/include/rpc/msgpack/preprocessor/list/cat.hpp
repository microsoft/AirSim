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
# ifndef MSGPACK_PREPROCESSOR_LIST_CAT_HPP
# define MSGPACK_PREPROCESSOR_LIST_CAT_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/list/fold_left.hpp>
#
# /* MSGPACK_PP_LIST_CAT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_CAT(list) MSGPACK_PP_LIST_FOLD_LEFT(MSGPACK_PP_LIST_CAT_O, MSGPACK_PP_LIST_FIRST(list), MSGPACK_PP_LIST_REST(list))
# else
#    define MSGPACK_PP_LIST_CAT(list) MSGPACK_PP_LIST_CAT_I(list)
#    define MSGPACK_PP_LIST_CAT_I(list) MSGPACK_PP_LIST_FOLD_LEFT(MSGPACK_PP_LIST_CAT_O, MSGPACK_PP_LIST_FIRST(list), MSGPACK_PP_LIST_REST(list))
# endif
#
# define MSGPACK_PP_LIST_CAT_O(d, s, x) MSGPACK_PP_CAT(s, x)
#
# /* MSGPACK_PP_LIST_CAT_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_CAT_D(d, list) MSGPACK_PP_LIST_FOLD_LEFT_ ## d(MSGPACK_PP_LIST_CAT_O, MSGPACK_PP_LIST_FIRST(list), MSGPACK_PP_LIST_REST(list))
# else
#    define MSGPACK_PP_LIST_CAT_D(d, list) MSGPACK_PP_LIST_CAT_D_I(d, list)
#    define MSGPACK_PP_LIST_CAT_D_I(d, list) MSGPACK_PP_LIST_FOLD_LEFT_ ## d(MSGPACK_PP_LIST_CAT_O, MSGPACK_PP_LIST_FIRST(list), MSGPACK_PP_LIST_REST(list))
# endif
#
# endif
