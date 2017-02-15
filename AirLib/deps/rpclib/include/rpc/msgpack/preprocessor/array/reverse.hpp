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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_REVERSE_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_REVERSE_HPP
#
# include <rpc/msgpack/preprocessor/array/data.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/reverse.hpp>
#
# /* MSGPACK_PP_ARRAY_REVERSE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_REVERSE(array) (MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_TUPLE_REVERSE(MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_ARRAY_DATA(array)))
# else
#    define MSGPACK_PP_ARRAY_REVERSE(array) MSGPACK_PP_ARRAY_REVERSE_I(array)
#    define MSGPACK_PP_ARRAY_REVERSE_I(array) (MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_TUPLE_REVERSE(MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_ARRAY_DATA(array)))
# endif
#
# endif
