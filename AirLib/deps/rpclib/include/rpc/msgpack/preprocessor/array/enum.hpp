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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_ENUM_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_ENUM_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_ARRAY_ENUM */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_ARRAY_ENUM(array) MSGPACK_PP_ARRAY_ENUM_I(MSGPACK_PP_TUPLE_REM_CTOR, array)
#    define MSGPACK_PP_ARRAY_ENUM_I(m, args) MSGPACK_PP_ARRAY_ENUM_II(m, args)
#    define MSGPACK_PP_ARRAY_ENUM_II(m, args) MSGPACK_PP_CAT(m ## args,)
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_ARRAY_ENUM(array) MSGPACK_PP_ARRAY_ENUM_I(array)
#    define MSGPACK_PP_ARRAY_ENUM_I(array) MSGPACK_PP_TUPLE_REM_CTOR ## array
# else
#    define MSGPACK_PP_ARRAY_ENUM(array) MSGPACK_PP_TUPLE_REM_CTOR array
# endif
#
# endif
