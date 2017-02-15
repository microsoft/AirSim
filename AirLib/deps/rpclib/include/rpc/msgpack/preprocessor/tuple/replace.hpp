# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2013.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_TUPLE_REPLACE_HPP
# define MSGPACK_PREPROCESSOR_TUPLE_REPLACE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if MSGPACK_PP_VARIADICS
#
# include <rpc/msgpack/preprocessor/array/replace.hpp>
# include <rpc/msgpack/preprocessor/array/to_tuple.hpp>
# include <rpc/msgpack/preprocessor/tuple/to_array.hpp>
#
# /* MSGPACK_PP_TUPLE_REPLACE */
#
# define MSGPACK_PP_TUPLE_REPLACE(tuple, i, elem) \
	MSGPACK_PP_ARRAY_TO_TUPLE(MSGPACK_PP_ARRAY_REPLACE(MSGPACK_PP_TUPLE_TO_ARRAY(tuple), i, elem)) \
/**/
#
# /* MSGPACK_PP_TUPLE_REPLACE_D */
#
# define MSGPACK_PP_TUPLE_REPLACE_D(d, tuple, i, elem) \
	MSGPACK_PP_ARRAY_TO_TUPLE(MSGPACK_PP_ARRAY_REPLACE_D(d, MSGPACK_PP_TUPLE_TO_ARRAY(tuple), i, elem)) \
/**/
#
# endif // MSGPACK_PP_VARIADICS
#
# endif // MSGPACK_PREPROCESSOR_TUPLE_REPLACE_HPP
