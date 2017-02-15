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
# ifndef MSGPACK_PREPROCESSOR_TUPLE_REMOVE_HPP
# define MSGPACK_PREPROCESSOR_TUPLE_REMOVE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if MSGPACK_PP_VARIADICS
#
# include <rpc/msgpack/preprocessor/array/remove.hpp>
# include <rpc/msgpack/preprocessor/array/to_tuple.hpp>
# include <rpc/msgpack/preprocessor/comparison/greater.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/tuple/size.hpp>
# include <rpc/msgpack/preprocessor/tuple/to_array.hpp>
#
# /* MSGPACK_PP_TUPLE_REMOVE */
#
# define MSGPACK_PP_TUPLE_REMOVE(tuple, i) \
	MSGPACK_PP_IIF \
		( \
		MSGPACK_PP_GREATER(MSGPACK_PP_TUPLE_SIZE(tuple),1), \
		MSGPACK_PP_TUPLE_REMOVE_EXEC, \
		MSGPACK_PP_TUPLE_REMOVE_RETURN \
		) \
	(tuple, i) \
/**/
#
# define MSGPACK_PP_TUPLE_REMOVE_EXEC(tuple, i) \
	MSGPACK_PP_ARRAY_TO_TUPLE(MSGPACK_PP_ARRAY_REMOVE(MSGPACK_PP_TUPLE_TO_ARRAY(tuple), i)) \
/**/
#
# define MSGPACK_PP_TUPLE_REMOVE_RETURN(tuple, i) tuple
#
# /* MSGPACK_PP_TUPLE_REMOVE_D */
#
# define MSGPACK_PP_TUPLE_REMOVE_D(d, tuple, i) \
	MSGPACK_PP_IIF \
		( \
		MSGPACK_PP_GREATER_D(d, MSGPACK_PP_TUPLE_SIZE(tuple), 1), \
		MSGPACK_PP_TUPLE_REMOVE_D_EXEC, \
		MSGPACK_PP_TUPLE_REMOVE_D_RETURN \
		) \
	(d, tuple, i) \
/**/
#
# define MSGPACK_PP_TUPLE_REMOVE_D_EXEC(d, tuple, i) \
	MSGPACK_PP_ARRAY_TO_TUPLE(MSGPACK_PP_ARRAY_REMOVE_D(d, MSGPACK_PP_TUPLE_TO_ARRAY(tuple), i)) \
/**/
#
# define MSGPACK_PP_TUPLE_REMOVE_D_RETURN(d, tuple, i) tuple
#
# endif // MSGPACK_PP_VARIADICS
#
# endif // MSGPACK_PREPROCESSOR_TUPLE_REMOVE_HPP
