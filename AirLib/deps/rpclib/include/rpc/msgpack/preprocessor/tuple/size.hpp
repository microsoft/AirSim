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
# ifndef MSGPACK_PREPROCESSOR_TUPLE_SIZE_HPP
# define MSGPACK_PREPROCESSOR_TUPLE_SIZE_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/variadic/size.hpp>
#
# if MSGPACK_PP_VARIADICS
#    if MSGPACK_PP_VARIADICS_MSVC
#        define MSGPACK_PP_TUPLE_SIZE(tuple) MSGPACK_PP_CAT(MSGPACK_PP_VARIADIC_SIZE tuple,)
#    else
#        define MSGPACK_PP_TUPLE_SIZE(tuple) MSGPACK_PP_VARIADIC_SIZE tuple
#    endif
# endif
#
# endif
