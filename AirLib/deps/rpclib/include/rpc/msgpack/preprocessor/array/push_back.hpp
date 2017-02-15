# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_ARRAY_PUSH_BACK_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_PUSH_BACK_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/array/data.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/punctuation/comma_if.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
# include <rpc/msgpack/preprocessor/array/detail/get_data.hpp>
#
# /* MSGPACK_PP_ARRAY_PUSH_BACK */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ARRAY_PUSH_BACK(array, elem) MSGPACK_PP_ARRAY_PUSH_BACK_I(MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_ARRAY_DATA(array), elem)
# else
#    define MSGPACK_PP_ARRAY_PUSH_BACK(array, elem) MSGPACK_PP_ARRAY_PUSH_BACK_D(array, elem)
#    define MSGPACK_PP_ARRAY_PUSH_BACK_D(array, elem) MSGPACK_PP_ARRAY_PUSH_BACK_I(MSGPACK_PP_ARRAY_SIZE(array), MSGPACK_PP_ARRAY_DATA(array), elem)
# endif
#
# define MSGPACK_PP_ARRAY_PUSH_BACK_I(size, data, elem) (MSGPACK_PP_INC(size), (MSGPACK_PP_ARRAY_DETAIL_GET_DATA(size,data) MSGPACK_PP_COMMA_IF(size) elem))
#
# endif
