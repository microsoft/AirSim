# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_VARIADIC_DETAIL_IS_SINGLE_RETURN_HPP
# define MSGPACK_PREPROCESSOR_VARIADIC_DETAIL_IS_SINGLE_RETURN_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_VARIADIC_IS_SINGLE_RETURN */
#
# if MSGPACK_PP_VARIADICS && MSGPACK_PP_VARIADICS_MSVC
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/facilities/is_1.hpp>
# include <rpc/msgpack/preprocessor/variadic/size.hpp>
# define MSGPACK_PP_VARIADIC_IS_SINGLE_RETURN(sr,nsr,...)	\
	MSGPACK_PP_IIF(MSGPACK_PP_IS_1(MSGPACK_PP_VARIADIC_SIZE(__VA_ARGS__)),sr,nsr) \
	/**/
# endif /* MSGPACK_PP_VARIADICS && MSGPACK_PP_VARIADICS_MSVC */
#
# endif /* MSGPACK_PREPROCESSOR_VARIADIC_DETAIL_IS_SINGLE_RETURN_HPP */
