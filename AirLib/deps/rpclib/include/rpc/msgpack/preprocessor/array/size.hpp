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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_SIZE_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_SIZE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_SIZE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_SIZE(array) MSGPACK_PP_TUPLE_ELEM(2, 0, array)
# else
#    define MSGPACK_PP_ARRAY_SIZE(array) MSGPACK_PP_ARRAY_SIZE_I(array)
#    define MSGPACK_PP_ARRAY_SIZE_I(array) MSGPACK_PP_ARRAY_SIZE_II array
#    define MSGPACK_PP_ARRAY_SIZE_II(size, data) size
# endif
#
# endif
