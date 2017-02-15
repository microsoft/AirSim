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
# ifndef MSGPACK_PREPROCESSOR_WSTRINGIZE_HPP
# define MSGPACK_PREPROCESSOR_WSTRINGIZE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_WSTRINGIZE */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_WSTRINGIZE(text) MSGPACK_PP_WSTRINGIZE_I(text)
# else
#    define MSGPACK_PP_WSTRINGIZE(text) MSGPACK_PP_WSTRINGIZE_OO((text))
#    define MSGPACK_PP_WSTRINGIZE_OO(par) MSGPACK_PP_WSTRINGIZE_I ## par
# endif
#
# define MSGPACK_PP_WSTRINGIZE_I(text) MSGPACK_PP_WSTRINGIZE_II(#text)
# define MSGPACK_PP_WSTRINGIZE_II(str) L ## str
#
# endif
