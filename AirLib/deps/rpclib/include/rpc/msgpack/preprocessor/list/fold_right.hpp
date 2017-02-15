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
# ifndef MSGPACK_PREPROCESSOR_LIST_FOLD_RIGHT_HPP
# define MSGPACK_PREPROCESSOR_LIST_FOLD_RIGHT_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/debug/error.hpp>
# include <rpc/msgpack/preprocessor/detail/auto_rec.hpp>
#
# if 0
#    define MSGPACK_PP_LIST_FOLD_RIGHT(op, state, list)
# endif
#
# define MSGPACK_PP_LIST_FOLD_RIGHT MSGPACK_PP_CAT(MSGPACK_PP_LIST_FOLD_RIGHT_, MSGPACK_PP_AUTO_REC(MSGPACK_PP_WHILE_P, 256))
#
# define MSGPACK_PP_LIST_FOLD_RIGHT_257(o, s, l) MSGPACK_PP_ERROR(0x0004)
#
# define MSGPACK_PP_LIST_FOLD_RIGHT_D(d, o, s, l) MSGPACK_PP_LIST_FOLD_RIGHT_ ## d(o, s, l)
# define MSGPACK_PP_LIST_FOLD_RIGHT_2ND MSGPACK_PP_LIST_FOLD_RIGHT
# define MSGPACK_PP_LIST_FOLD_RIGHT_2ND_D MSGPACK_PP_LIST_FOLD_RIGHT_D
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    include <rpc/msgpack/preprocessor/list/detail/edg/fold_right.hpp>
# else
#    include <rpc/msgpack/preprocessor/list/detail/fold_right.hpp>
# endif
#
# endif
