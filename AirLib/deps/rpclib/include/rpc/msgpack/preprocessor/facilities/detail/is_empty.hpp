# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
#ifndef MSGPACK_PREPROCESSOR_DETAIL_IS_EMPTY_HPP
#define MSGPACK_PREPROCESSOR_DETAIL_IS_EMPTY_HPP

#include <rpc/msgpack/preprocessor/punctuation/is_begin_parens.hpp>

#if MSGPACK_PP_VARIADICS_MSVC

# pragma warning(once:4002)

#define MSGPACK_PP_DETAIL_IS_EMPTY_IIF_0(t, b) b
#define MSGPACK_PP_DETAIL_IS_EMPTY_IIF_1(t, b) t

#else

#define MSGPACK_PP_DETAIL_IS_EMPTY_IIF_0(t, ...) __VA_ARGS__
#define MSGPACK_PP_DETAIL_IS_EMPTY_IIF_1(t, ...) t

#endif

#if MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400

#define MSGPACK_PP_DETAIL_IS_EMPTY_PROCESS(param) \
	MSGPACK_PP_IS_BEGIN_PARENS \
    	( \
        MSGPACK_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C param () \
        ) \
/**/

#else

#define MSGPACK_PP_DETAIL_IS_EMPTY_PROCESS(...) \
	MSGPACK_PP_IS_BEGIN_PARENS \
        ( \
        MSGPACK_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C __VA_ARGS__ () \
        ) \
/**/

#endif

#define MSGPACK_PP_DETAIL_IS_EMPTY_PRIMITIVE_CAT(a, b) a ## b
#define MSGPACK_PP_DETAIL_IS_EMPTY_IIF(bit) MSGPACK_PP_DETAIL_IS_EMPTY_PRIMITIVE_CAT(MSGPACK_PP_DETAIL_IS_EMPTY_IIF_,bit)
#define MSGPACK_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C(...) ()

#endif /* MSGPACK_PREPROCESSOR_DETAIL_IS_EMPTY_HPP */
