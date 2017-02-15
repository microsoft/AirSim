# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2011.                                  *
#  *     (C) Copyright Edward Diener 2011.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_OVERLOAD_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_OVERLOAD_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/variadic/size.hpp>
#
# /* MSGPACK_PP_OVERLOAD */
#
# if MSGPACK_PP_VARIADICS
#    define MSGPACK_PP_OVERLOAD(prefix, ...) MSGPACK_PP_CAT(prefix, MSGPACK_PP_VARIADIC_SIZE(__VA_ARGS__))
# endif
#
# endif
