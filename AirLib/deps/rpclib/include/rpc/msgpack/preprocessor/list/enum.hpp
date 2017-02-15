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
# ifndef MSGPACK_PREPROCESSOR_LIST_ENUM_HPP
# define MSGPACK_PREPROCESSOR_LIST_ENUM_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/for_each_i.hpp>
# include <rpc/msgpack/preprocessor/punctuation/comma_if.hpp>
#
# /* MSGPACK_PP_LIST_ENUM */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_ENUM(list) MSGPACK_PP_LIST_FOR_EACH_I(MSGPACK_PP_LIST_ENUM_O, MSGPACK_PP_NIL, list)
# else
#    define MSGPACK_PP_LIST_ENUM(list) MSGPACK_PP_LIST_ENUM_I(list)
#    define MSGPACK_PP_LIST_ENUM_I(list) MSGPACK_PP_LIST_FOR_EACH_I(MSGPACK_PP_LIST_ENUM_O, MSGPACK_PP_NIL, list)
# endif
#
# define MSGPACK_PP_LIST_ENUM_O(r, _, i, elem) MSGPACK_PP_COMMA_IF(i) elem
#
# /* MSGPACK_PP_LIST_ENUM_R */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_ENUM_R(r, list) MSGPACK_PP_LIST_FOR_EACH_I_R(r, MSGPACK_PP_LIST_ENUM_O, MSGPACK_PP_NIL, list)
# else
#    define MSGPACK_PP_LIST_ENUM_R(r, list) MSGPACK_PP_LIST_ENUM_R_I(r, list)
#    define MSGPACK_PP_LIST_ENUM_R_I(r, list) MSGPACK_PP_LIST_FOR_EACH_I_R(r, MSGPACK_PP_LIST_ENUM_O, MSGPACK_PP_NIL, list)
# endif
#
# endif
