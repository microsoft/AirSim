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
# ifndef MSGPACK_PREPROCESSOR_CONTROL_IF_HPP
# define MSGPACK_PREPROCESSOR_CONTROL_IF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/logical/bool.hpp>
#
# /* MSGPACK_PP_IF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_IF(cond, t, f) MSGPACK_PP_IIF(MSGPACK_PP_BOOL(cond), t, f)
# else
#    define MSGPACK_PP_IF(cond, t, f) MSGPACK_PP_IF_I(cond, t, f)
#    define MSGPACK_PP_IF_I(cond, t, f) MSGPACK_PP_IIF(MSGPACK_PP_BOOL(cond), t, f)
# endif
#
# endif
