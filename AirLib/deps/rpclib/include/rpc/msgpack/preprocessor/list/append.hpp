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
# ifndef MSGPACK_PREPROCESSOR_LIST_APPEND_HPP
# define MSGPACK_PREPROCESSOR_LIST_APPEND_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/fold_right.hpp>
#
# /* MSGPACK_PP_LIST_APPEND */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_APPEND(a, b) MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_APPEND_O, b, a)
# else
#    define MSGPACK_PP_LIST_APPEND(a, b) MSGPACK_PP_LIST_APPEND_I(a, b)
#    define MSGPACK_PP_LIST_APPEND_I(a, b) MSGPACK_PP_LIST_FOLD_RIGHT(MSGPACK_PP_LIST_APPEND_O, b, a)
# endif
#
# define MSGPACK_PP_LIST_APPEND_O(d, s, x) (x, s)
#
# /* MSGPACK_PP_LIST_APPEND_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_APPEND_D(d, a, b) MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_APPEND_O, b, a)
# else
#    define MSGPACK_PP_LIST_APPEND_D(d, a, b) MSGPACK_PP_LIST_APPEND_D_I(d, a, b)
#    define MSGPACK_PP_LIST_APPEND_D_I(d, a, b) MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(MSGPACK_PP_LIST_APPEND_O, b, a)
# endif
#
# endif
