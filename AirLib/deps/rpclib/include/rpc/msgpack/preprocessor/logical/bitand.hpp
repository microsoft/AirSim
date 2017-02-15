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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_BITAND_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_BITAND_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_BITAND */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_BITAND(x, y) MSGPACK_PP_BITAND_I(x, y)
# else
#    define MSGPACK_PP_BITAND(x, y) MSGPACK_PP_BITAND_OO((x, y))
#    define MSGPACK_PP_BITAND_OO(par) MSGPACK_PP_BITAND_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_BITAND_I(x, y) MSGPACK_PP_BITAND_ ## x ## y
# else
#    define MSGPACK_PP_BITAND_I(x, y) MSGPACK_PP_BITAND_ID(MSGPACK_PP_BITAND_ ## x ## y)
#    define MSGPACK_PP_BITAND_ID(res) res
# endif
#
# define MSGPACK_PP_BITAND_00 0
# define MSGPACK_PP_BITAND_01 0
# define MSGPACK_PP_BITAND_10 0
# define MSGPACK_PP_BITAND_11 1
#
# endif
