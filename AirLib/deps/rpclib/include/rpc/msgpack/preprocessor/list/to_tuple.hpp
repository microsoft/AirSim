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
# ifndef MSGPACK_PREPROCESSOR_LIST_TO_TUPLE_HPP
# define MSGPACK_PREPROCESSOR_LIST_TO_TUPLE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/enum.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
#
# /* MSGPACK_PP_LIST_TO_TUPLE */
#
# define MSGPACK_PP_LIST_TO_TUPLE(list) \
	MSGPACK_PP_IIF \
		( \
		MSGPACK_PP_LIST_IS_NIL(list), \
		MSGPACK_PP_LIST_TO_TUPLE_EMPTY, \
		MSGPACK_PP_LIST_TO_TUPLE_DO \
		) \
	(list) \
/**/
# define MSGPACK_PP_LIST_TO_TUPLE_EMPTY(list)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_TO_TUPLE_DO(list) (MSGPACK_PP_LIST_ENUM(list))
# else
#    define MSGPACK_PP_LIST_TO_TUPLE_DO(list) MSGPACK_PP_LIST_TO_TUPLE_I(list)
#    define MSGPACK_PP_LIST_TO_TUPLE_I(list) (MSGPACK_PP_LIST_ENUM(list))
# endif
#
# /* MSGPACK_PP_LIST_TO_TUPLE_R */
#
# define MSGPACK_PP_LIST_TO_TUPLE_R(r, list) \
	MSGPACK_PP_IIF \
		( \
		MSGPACK_PP_LIST_IS_NIL(list), \
		MSGPACK_PP_LIST_TO_TUPLE_R_EMPTY, \
		MSGPACK_PP_LIST_TO_TUPLE_R_DO \
		) \
	(r, list) \
/**/
# define MSGPACK_PP_LIST_TO_TUPLE_R_EMPTY(r,list)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_TO_TUPLE_R_DO(r, list) (MSGPACK_PP_LIST_ENUM_R(r, list))
# else
#    define MSGPACK_PP_LIST_TO_TUPLE_R_DO(r, list) MSGPACK_PP_LIST_TO_TUPLE_R_I(r, list)
#    define MSGPACK_PP_LIST_TO_TUPLE_R_I(r, list) (MSGPACK_PP_LIST_ENUM_R(r, list))
# endif
#
# endif
