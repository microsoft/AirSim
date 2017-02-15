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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_TO_TUPLE_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_TO_TUPLE_HPP
#
# include <rpc/msgpack/preprocessor/array/data.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
#
# /* MSGPACK_PP_ARRAY_TO_TUPLE */
#
#    define MSGPACK_PP_ARRAY_TO_TUPLE(array) \
		MSGPACK_PP_IF \
			( \
			MSGPACK_PP_ARRAY_SIZE(array), \
			MSGPACK_PP_ARRAY_DATA, \
			MSGPACK_PP_ARRAY_TO_TUPLE_EMPTY \
			) \
		(array) \
/**/
#    define MSGPACK_PP_ARRAY_TO_TUPLE_EMPTY(array)
#
# endif
