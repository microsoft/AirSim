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
# ifndef MSGPACK_PREPROCESSOR_STRINGIZE_HPP
# define MSGPACK_PREPROCESSOR_STRINGIZE_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_STRINGIZE */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_STRINGIZE(text) MSGPACK_PP_STRINGIZE_A((text))
#    define MSGPACK_PP_STRINGIZE_A(arg) MSGPACK_PP_STRINGIZE_I arg
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_STRINGIZE(text) MSGPACK_PP_STRINGIZE_OO((text))
#    define MSGPACK_PP_STRINGIZE_OO(par) MSGPACK_PP_STRINGIZE_I ## par
# else
#    define MSGPACK_PP_STRINGIZE(text) MSGPACK_PP_STRINGIZE_I(text)
# endif
#
# define MSGPACK_PP_STRINGIZE_I(text) #text
#
# endif
