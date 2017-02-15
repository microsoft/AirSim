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
#ifndef MSGPACK_PREPROCESSOR_DETAIL_IS_BEGIN_PARENS_HPP
#define MSGPACK_PREPROCESSOR_DETAIL_IS_BEGIN_PARENS_HPP

#if MSGPACK_PP_VARIADICS_MSVC

#include <rpc/msgpack/preprocessor/facilities/empty.hpp>

#define MSGPACK_PP_DETAIL_VD_IBP_CAT(a, b) MSGPACK_PP_DETAIL_VD_IBP_CAT_I(a, b)
#define MSGPACK_PP_DETAIL_VD_IBP_CAT_I(a, b) MSGPACK_PP_DETAIL_VD_IBP_CAT_II(a ## b)
#define MSGPACK_PP_DETAIL_VD_IBP_CAT_II(res) res

#define MSGPACK_PP_DETAIL_IBP_SPLIT(i, ...) \
    MSGPACK_PP_DETAIL_VD_IBP_CAT(MSGPACK_PP_DETAIL_IBP_PRIMITIVE_CAT(MSGPACK_PP_DETAIL_IBP_SPLIT_,i)(__VA_ARGS__),MSGPACK_PP_EMPTY()) \
/**/

#define MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_C(...) 1 1

#else

#define MSGPACK_PP_DETAIL_IBP_SPLIT(i, ...) \
    MSGPACK_PP_DETAIL_IBP_PRIMITIVE_CAT(MSGPACK_PP_DETAIL_IBP_SPLIT_,i)(__VA_ARGS__) \
/**/

#define MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_C(...) 1

#endif /* MSGPACK_PP_VARIADICS_MSVC */

#define MSGPACK_PP_DETAIL_IBP_SPLIT_0(a, ...) a
#define MSGPACK_PP_DETAIL_IBP_SPLIT_1(a, ...) __VA_ARGS__

#define MSGPACK_PP_DETAIL_IBP_CAT(a, ...) MSGPACK_PP_DETAIL_IBP_PRIMITIVE_CAT(a,__VA_ARGS__)
#define MSGPACK_PP_DETAIL_IBP_PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

#define MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_R_1 1,
#define MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_R_MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_C 0,

#endif /* MSGPACK_PREPROCESSOR_DETAIL_IS_BEGIN_PARENS_HPP */
