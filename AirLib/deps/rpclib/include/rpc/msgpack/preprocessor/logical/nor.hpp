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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_NOR_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_NOR_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/logical/bool.hpp>
# include <rpc/msgpack/preprocessor/logical/bitnor.hpp>
#
# /* MSGPACK_PP_NOR */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_NOR(p, q) MSGPACK_PP_BITNOR(MSGPACK_PP_BOOL(p), MSGPACK_PP_BOOL(q))
# else
#    define MSGPACK_PP_NOR(p, q) MSGPACK_PP_NOR_I(p, q)
#    define MSGPACK_PP_NOR_I(p, q) MSGPACK_PP_BITNOR(MSGPACK_PP_BOOL(p), MSGPACK_PP_BOOL(q))
# endif
#
# endif
