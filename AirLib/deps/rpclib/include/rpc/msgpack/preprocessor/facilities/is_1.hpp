# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2003.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_IS_1_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_IS_1_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/facilities/is_empty.hpp>
#
# /* MSGPACK_PP_IS_1 */
#
# define MSGPACK_PP_IS_1(x) MSGPACK_PP_IS_EMPTY(MSGPACK_PP_CAT(MSGPACK_PP_IS_1_HELPER_, x))
# define MSGPACK_PP_IS_1_HELPER_1
#
# endif
