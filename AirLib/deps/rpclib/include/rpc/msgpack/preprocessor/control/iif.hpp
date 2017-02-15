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
# ifndef MSGPACK_PREPROCESSOR_CONTROL_IIF_HPP
# define MSGPACK_PREPROCESSOR_CONTROL_IIF_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_IIF(bit, t, f) MSGPACK_PP_IIF_I(bit, t, f)
# else
#    define MSGPACK_PP_IIF(bit, t, f) MSGPACK_PP_IIF_OO((bit, t, f))
#    define MSGPACK_PP_IIF_OO(par) MSGPACK_PP_IIF_I ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_IIF_I(bit, t, f) MSGPACK_PP_IIF_ ## bit(t, f)
# else
#    define MSGPACK_PP_IIF_I(bit, t, f) MSGPACK_PP_IIF_II(MSGPACK_PP_IIF_ ## bit(t, f))
#    define MSGPACK_PP_IIF_II(id) id
# endif
#
# define MSGPACK_PP_IIF_0(t, f) f
# define MSGPACK_PP_IIF_1(t, f) t
#
# endif
