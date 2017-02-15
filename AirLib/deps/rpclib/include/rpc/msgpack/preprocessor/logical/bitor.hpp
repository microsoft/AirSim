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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_BITOR_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_BITOR_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_BITOR */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_BITOR(x, y) MSGPACK_PP_BITOR_I(x, y)
# else
#    define MSGPACK_PP_BITOR(x, y) MSGPACK_PP_BITOR_OO((x, y))
#    define MSGPACK_PP_BITOR_OO(par) MSGPACK_PP_BITOR_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_BITOR_I(x, y) MSGPACK_PP_BITOR_ ## x ## y
# else
#    define MSGPACK_PP_BITOR_I(x, y) MSGPACK_PP_BITOR_ID(MSGPACK_PP_BITOR_ ## x ## y)
#    define MSGPACK_PP_BITOR_ID(id) id
# endif
#
# define MSGPACK_PP_BITOR_00 0
# define MSGPACK_PP_BITOR_01 1
# define MSGPACK_PP_BITOR_10 1
# define MSGPACK_PP_BITOR_11 1
#
# endif
