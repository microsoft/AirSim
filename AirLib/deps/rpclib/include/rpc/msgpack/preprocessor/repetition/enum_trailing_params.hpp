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
# ifndef MSGPACK_PREPROCESSOR_REPETITION_ENUM_TRAILING_PARAMS_HPP
# define MSGPACK_PREPROCESSOR_REPETITION_ENUM_TRAILING_PARAMS_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/repetition/repeat.hpp>
#
# /* MSGPACK_PP_ENUM_TRAILING_PARAMS */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS(count, param) MSGPACK_PP_REPEAT(count, MSGPACK_PP_ENUM_TRAILING_PARAMS_M, param)
# else
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS(count, param) MSGPACK_PP_ENUM_TRAILING_PARAMS_I(count, param)
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS_I(count, param) MSGPACK_PP_REPEAT(count, MSGPACK_PP_ENUM_TRAILING_PARAMS_M, param)
# endif
#
# define MSGPACK_PP_ENUM_TRAILING_PARAMS_M(z, n, param) , param ## n
#
# /* MSGPACK_PP_ENUM_TRAILING_PARAMS_Z */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS_Z(z, count, param) MSGPACK_PP_REPEAT_ ## z(count, MSGPACK_PP_ENUM_TRAILING_PARAMS_M, param)
# else
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS_Z(z, count, param) MSGPACK_PP_ENUM_TRAILING_PARAMS_Z_I(z, count, param)
#    define MSGPACK_PP_ENUM_TRAILING_PARAMS_Z_I(z, count, param) MSGPACK_PP_REPEAT_ ## z(count, MSGPACK_PP_ENUM_TRAILING_PARAMS_M, param)
# endif
#
# endif
