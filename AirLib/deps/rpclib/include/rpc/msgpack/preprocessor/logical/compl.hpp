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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_COMPL_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_COMPL_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_COMPL */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_COMPL(x) MSGPACK_PP_COMPL_I(x)
# else
#    define MSGPACK_PP_COMPL(x) MSGPACK_PP_COMPL_OO((x))
#    define MSGPACK_PP_COMPL_OO(par) MSGPACK_PP_COMPL_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_COMPL_I(x) MSGPACK_PP_COMPL_ ## x
# else
#    define MSGPACK_PP_COMPL_I(x) MSGPACK_PP_COMPL_ID(MSGPACK_PP_COMPL_ ## x)
#    define MSGPACK_PP_COMPL_ID(id) id
# endif
#
# define MSGPACK_PP_COMPL_0 1
# define MSGPACK_PP_COMPL_1 0
#
# endif
