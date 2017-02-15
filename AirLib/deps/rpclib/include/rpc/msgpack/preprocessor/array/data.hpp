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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_DATA_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_DATA_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ARRAY_DATA */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_DATA(array) MSGPACK_PP_TUPLE_ELEM(2, 1, array)
# else
#    define MSGPACK_PP_ARRAY_DATA(array) MSGPACK_PP_ARRAY_DATA_I(array)
#    define MSGPACK_PP_ARRAY_DATA_I(array) MSGPACK_PP_ARRAY_DATA_II array
#    define MSGPACK_PP_ARRAY_DATA_II(size, data) data
# endif
#
# endif
