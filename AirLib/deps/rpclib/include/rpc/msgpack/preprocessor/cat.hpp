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
# ifndef MSGPACK_PREPROCESSOR_CAT_HPP
# define MSGPACK_PREPROCESSOR_CAT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_CAT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_CAT(a, b) MSGPACK_PP_CAT_I(a, b)
# else
#    define MSGPACK_PP_CAT(a, b) MSGPACK_PP_CAT_OO((a, b))
#    define MSGPACK_PP_CAT_OO(par) MSGPACK_PP_CAT_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_CAT_I(a, b) a ## b
# else
#    define MSGPACK_PP_CAT_I(a, b) MSGPACK_PP_CAT_II(~, a ## b)
#    define MSGPACK_PP_CAT_II(p, res) res
# endif
#
# endif
