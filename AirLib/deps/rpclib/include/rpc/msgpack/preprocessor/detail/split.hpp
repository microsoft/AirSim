# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# ifndef MSGPACK_PREPROCESSOR_DETAIL_SPLIT_HPP
# define MSGPACK_PREPROCESSOR_DETAIL_SPLIT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_SPLIT */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_SPLIT(n, im) MSGPACK_PP_SPLIT_I((n, im))
#    define MSGPACK_PP_SPLIT_I(par) MSGPACK_PP_SPLIT_II ## par
#    define MSGPACK_PP_SPLIT_II(n, a, b) MSGPACK_PP_SPLIT_ ## n(a, b)
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_SPLIT(n, im) MSGPACK_PP_SPLIT_I(n((im)))
#    define MSGPACK_PP_SPLIT_I(n) MSGPACK_PP_SPLIT_ID(MSGPACK_PP_SPLIT_II_ ## n)
#    define MSGPACK_PP_SPLIT_II_0(s) MSGPACK_PP_SPLIT_ID(MSGPACK_PP_SPLIT_0 s)
#    define MSGPACK_PP_SPLIT_II_1(s) MSGPACK_PP_SPLIT_ID(MSGPACK_PP_SPLIT_1 s)
#    define MSGPACK_PP_SPLIT_ID(id) id
# else
#    define MSGPACK_PP_SPLIT(n, im) MSGPACK_PP_SPLIT_I(n)(im)
#    define MSGPACK_PP_SPLIT_I(n) MSGPACK_PP_SPLIT_ ## n
# endif
#
# define MSGPACK_PP_SPLIT_0(a, b) a
# define MSGPACK_PP_SPLIT_1(a, b) b
#
# endif
