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
# ifndef MSGPACK_PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
# define MSGPACK_PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
# include <rpc/msgpack/preprocessor/facilities/empty.hpp>
# include <rpc/msgpack/preprocessor/punctuation/comma.hpp>
#
# /* MSGPACK_PP_COMMA_IF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_COMMA_IF(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_COMMA, MSGPACK_PP_EMPTY)()
# else
#    define MSGPACK_PP_COMMA_IF(cond) MSGPACK_PP_COMMA_IF_I(cond)
#    define MSGPACK_PP_COMMA_IF_I(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_COMMA, MSGPACK_PP_EMPTY)()
# endif
#
# endif
