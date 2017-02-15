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
# ifndef MSGPACK_PREPROCESSOR_PUNCTUATION_PAREN_IF_HPP
# define MSGPACK_PREPROCESSOR_PUNCTUATION_PAREN_IF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/control/if.hpp>
# include <rpc/msgpack/preprocessor/facilities/empty.hpp>
# include <rpc/msgpack/preprocessor/punctuation/paren.hpp>
#
# /* MSGPACK_PP_LPAREN_IF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LPAREN_IF(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_LPAREN, MSGPACK_PP_EMPTY)()
# else
#    define MSGPACK_PP_LPAREN_IF(cond) MSGPACK_PP_LPAREN_IF_I(cond)
#    define MSGPACK_PP_LPAREN_IF_I(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_LPAREN, MSGPACK_PP_EMPTY)()
# endif
#
# /* MSGPACK_PP_RPAREN_IF */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_RPAREN_IF(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_RPAREN, MSGPACK_PP_EMPTY)()
# else
#    define MSGPACK_PP_RPAREN_IF(cond) MSGPACK_PP_RPAREN_IF_I(cond)
#    define MSGPACK_PP_RPAREN_IF_I(cond) MSGPACK_PP_IF(cond, MSGPACK_PP_RPAREN, MSGPACK_PP_EMPTY)()
# endif
#
# endif
