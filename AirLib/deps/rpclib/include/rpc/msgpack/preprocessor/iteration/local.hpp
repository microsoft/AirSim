# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_ITERATION_LOCAL_HPP
# define MSGPACK_PREPROCESSOR_ITERATION_LOCAL_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/slot/slot.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_LOCAL_ITERATE */
#
# define MSGPACK_PP_LOCAL_ITERATE() <msgpack/preprocessor/iteration/detail/local.hpp>
#
# define MSGPACK_PP_LOCAL_C(n) (MSGPACK_PP_LOCAL_S) <= n && (MSGPACK_PP_LOCAL_F) >= n
# define MSGPACK_PP_LOCAL_R(n) (MSGPACK_PP_LOCAL_F) <= n && (MSGPACK_PP_LOCAL_S) >= n
#
# endif
