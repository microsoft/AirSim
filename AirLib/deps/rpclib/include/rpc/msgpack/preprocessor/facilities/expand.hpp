# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_EXPAND_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_EXPAND_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC() && ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_DMC()
#    define MSGPACK_PP_EXPAND(x) MSGPACK_PP_EXPAND_I(x)
# else
#    define MSGPACK_PP_EXPAND(x) MSGPACK_PP_EXPAND_OO((x))
#    define MSGPACK_PP_EXPAND_OO(par) MSGPACK_PP_EXPAND_I ## par
# endif
#
# define MSGPACK_PP_EXPAND_I(x) x
#
# endif
