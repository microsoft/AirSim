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
# ifndef MSGPACK_PREPROCESSOR_IS_BEGIN_PARENS_HPP
# define MSGPACK_PREPROCESSOR_IS_BEGIN_PARENS_HPP

# include <rpc/msgpack/preprocessor/config/config.hpp>

#if MSGPACK_PP_VARIADICS

#include <rpc/msgpack/preprocessor/punctuation/detail/is_begin_parens.hpp>

#if MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400

#define MSGPACK_PP_IS_BEGIN_PARENS(param) \
    MSGPACK_PP_DETAIL_IBP_SPLIT \
      ( \
      0, \
      MSGPACK_PP_DETAIL_IBP_CAT \
        ( \
        MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_R_, \
        MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_C param \
        ) \
      ) \
/**/

#else

#define MSGPACK_PP_IS_BEGIN_PARENS(...) \
    MSGPACK_PP_DETAIL_IBP_SPLIT \
      ( \
      0, \
      MSGPACK_PP_DETAIL_IBP_CAT \
        ( \
        MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_R_, \
        MSGPACK_PP_DETAIL_IBP_IS_VARIADIC_C __VA_ARGS__ \
        ) \
      ) \
/**/

#endif /* MSGPACK_PP_VARIADICS_MSVC && _MSC_VER <= 1400 */
#endif /* MSGPACK_PP_VARIADICS */
#endif /* MSGPACK_PREPROCESSOR_IS_BEGIN_PARENS_HPP */
