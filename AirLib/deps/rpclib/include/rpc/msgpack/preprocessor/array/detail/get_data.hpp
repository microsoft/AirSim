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
# ifndef MSGPACK_PREPROCESSOR_ARRAY_DETAIL_GET_DATA_HPP
# define MSGPACK_PREPROCESSOR_ARRAY_DETAIL_GET_DATA_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/facilities/is_1.hpp>
#
# /* MSGPACK_PP_ARRAY_DETAIL_GET_DATA */
#
# define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_NONE(size, data)

# if MSGPACK_PP_VARIADICS && !(MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400)
# 	 if MSGPACK_PP_VARIADICS_MSVC
# 		define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY_VC_DEFAULT(size, data) MSGPACK_PP_TUPLE_REM(size) data
# 		define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY_VC_CAT(size, data) MSGPACK_PP_TUPLE_REM_CAT(size) data
# 		define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY(size, data) \
			MSGPACK_PP_IIF \
				( \
				MSGPACK_PP_IS_1(size), \
				MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY_VC_CAT, \
				MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY_VC_DEFAULT \
				) \
			(size,data) \
/**/
#    else
# 		define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY(size, data) MSGPACK_PP_TUPLE_REM(size) data
#    endif
# else
# 	 define MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY(size, data) MSGPACK_PP_TUPLE_REM(size) data
# endif

# define MSGPACK_PP_ARRAY_DETAIL_GET_DATA(size, data) \
	MSGPACK_PP_IF \
		( \
		size, \
		MSGPACK_PP_ARRAY_DETAIL_GET_DATA_ANY, \
		MSGPACK_PP_ARRAY_DETAIL_GET_DATA_NONE \
		) \
	(size,data) \
/**/
#
# endif /* MSGPACK_PREPROCESSOR_ARRAY_DETAIL_GET_DATA_HPP */
