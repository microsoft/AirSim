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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_BITXOR_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_BITXOR_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_BITXOR */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_BITXOR(x, y) MSGPACK_PP_BITXOR_I(x, y)
# else
#    define MSGPACK_PP_BITXOR(x, y) MSGPACK_PP_BITXOR_OO((x, y))
#    define MSGPACK_PP_BITXOR_OO(par) MSGPACK_PP_BITXOR_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_BITXOR_I(x, y) MSGPACK_PP_BITXOR_ ## x ## y
# else
#    define MSGPACK_PP_BITXOR_I(x, y) MSGPACK_PP_BITXOR_ID(MSGPACK_PP_BITXOR_ ## x ## y)
#    define MSGPACK_PP_BITXOR_ID(id) id
# endif
#
# define MSGPACK_PP_BITXOR_00 0
# define MSGPACK_PP_BITXOR_01 1
# define MSGPACK_PP_BITXOR_10 1
# define MSGPACK_PP_BITXOR_11 0
#
# endif
