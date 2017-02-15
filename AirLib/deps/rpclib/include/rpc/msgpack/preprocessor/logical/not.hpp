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
# ifndef MSGPACK_PREPROCESSOR_LOGICAL_NOT_HPP
# define MSGPACK_PREPROCESSOR_LOGICAL_NOT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/logical/bool.hpp>
# include <rpc/msgpack/preprocessor/logical/compl.hpp>
#
# /* MSGPACK_PP_NOT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_NOT(x) MSGPACK_PP_COMPL(MSGPACK_PP_BOOL(x))
# else
#    define MSGPACK_PP_NOT(x) MSGPACK_PP_NOT_I(x)
#    define MSGPACK_PP_NOT_I(x) MSGPACK_PP_COMPL(MSGPACK_PP_BOOL(x))
# endif
#
# endif
