# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2011.                                    *
#  *     (C) Copyright Paul Mensonides 2011.                                  *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_ARRAY_TO_LIST_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_TO_LIST_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
# include <rpc/msgpack/preprocessor/tuple/to_list.hpp>
#
# /* MSGPACK_PP_ARRAY_TO_LIST */
#
#    define MSGPACK_PP_ARRAY_TO_LIST(array) \
		MSGPACK_PP_IF \
			( \
			MSGPACK_PP_ARRAY_SIZE(array), \
			MSGPACK_PP_ARRAY_TO_LIST_DO, \
			MSGPACK_PP_ARRAY_TO_LIST_EMPTY \
			) \
		(array) \
/**/
#
#    define MSGPACK_PP_ARRAY_TO_LIST_EMPTY(array) MSGPACK_PP_NIL
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_ARRAY_TO_LIST_DO(array) MSGPACK_PP_ARRAY_TO_LIST_I(MSGPACK_PP_TUPLE_TO_LIST, array)
#    define MSGPACK_PP_ARRAY_TO_LIST_I(m, args) MSGPACK_PP_ARRAY_TO_LIST_II(m, args)
#    define MSGPACK_PP_ARRAY_TO_LIST_II(m, args) MSGPACK_PP_CAT(m ## args,)
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_ARRAY_TO_LIST_DO(array) MSGPACK_PP_ARRAY_TO_LIST_I(array)
#    define MSGPACK_PP_ARRAY_TO_LIST_I(array) MSGPACK_PP_TUPLE_TO_LIST ## array
# else
#    define MSGPACK_PP_ARRAY_TO_LIST_DO(array) MSGPACK_PP_TUPLE_TO_LIST array
# endif
#
# endif
