# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_ARRAY_ELEM_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_ELEM_HPP
#
# include <rpc/msgpack/preprocessor/array/data.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_ELEM */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_ELEM(i, array) MSGPACK_PP_TUPLE_ELEM(MSGPACK_PP_ARRAY_SIZE(array), i, MSGPACK_PP_ARRAY_DATA(array))
# else
#    define MSGPACK_PP_ARRAY_ELEM(i, array) MSGPACK_PP_ARRAY_ELEM_I(i, array)
#    define MSGPACK_PP_ARRAY_ELEM_I(i, array) MSGPACK_PP_TUPLE_ELEM(MSGPACK_PP_ARRAY_SIZE(array), i, MSGPACK_PP_ARRAY_DATA(array))
# endif
#
# endif
