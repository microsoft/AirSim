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
# ifndef MSGPACK_PREPROCESSOR_VARIADIC_TO_ARRAY_HPP
# define MSGPACK_PREPROCESSOR_VARIADIC_TO_ARRAY_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/to_array.hpp>
# if MSGPACK_PP_VARIADICS_MSVC
#    include <rpc/msgpack/preprocessor/variadic/size.hpp>
# endif
#
# /* MSGPACK_PP_VARIADIC_TO_ARRAY */
#
# if MSGPACK_PP_VARIADICS
#    if MSGPACK_PP_VARIADICS_MSVC
#        define MSGPACK_PP_VARIADIC_TO_ARRAY(...) MSGPACK_PP_TUPLE_TO_ARRAY_2(MSGPACK_PP_VARIADIC_SIZE(__VA_ARGS__),(__VA_ARGS__))
#    else
#        define MSGPACK_PP_VARIADIC_TO_ARRAY(...) MSGPACK_PP_TUPLE_TO_ARRAY((__VA_ARGS__))
#    endif
# endif
#
# endif
