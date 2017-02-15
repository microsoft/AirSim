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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_BITNOR_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_BITNOR_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_BITNOR */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_BITNOR(x, y) MSGPACK_PP_BITNOR_I(x, y)
# else
#    define MSGPACK_PP_BITNOR(x, y) MSGPACK_PP_BITNOR_OO((x, y))
#    define MSGPACK_PP_BITNOR_OO(par) MSGPACK_PP_BITNOR_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_BITNOR_I(x, y) MSGPACK_PP_BITNOR_ ## x ## y
# else
#    define MSGPACK_PP_BITNOR_I(x, y) MSGPACK_PP_BITNOR_ID(MSGPACK_PP_BITNOR_ ## x ## y)
#    define MSGPACK_PP_BITNOR_ID(id) id
# endif
#
# define MSGPACK_PP_BITNOR_00 1
# define MSGPACK_PP_BITNOR_01 0
# define MSGPACK_PP_BITNOR_10 0
# define MSGPACK_PP_BITNOR_11 0
#
# endif
