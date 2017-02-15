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
# ifndef MSGPACK_PREPROCESSOR_LIST_AT_HPP
# define MSGPACK_PREPROCESSOR_LIST_AT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/adt.hpp>
# include <rpc/msgpack/preprocessor/list/rest_n.hpp>
#
# /* MSGPACK_PP_LIST_AT */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_AT(list, index) MSGPACK_PP_LIST_FIRST(MSGPACK_PP_LIST_REST_N(index, list))
# else
#    define MSGPACK_PP_LIST_AT(list, index) MSGPACK_PP_LIST_AT_I(list, index)
#    define MSGPACK_PP_LIST_AT_I(list, index) MSGPACK_PP_LIST_FIRST(MSGPACK_PP_LIST_REST_N(index, list))
# endif
#
# /* MSGPACK_PP_LIST_AT_D */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_AT_D(d, list, index) MSGPACK_PP_LIST_FIRST(MSGPACK_PP_LIST_REST_N_D(d, index, list))
# else
#    define MSGPACK_PP_LIST_AT_D(d, list, index) MSGPACK_PP_LIST_AT_D_I(d, list, index)
#    define MSGPACK_PP_LIST_AT_D_I(d, list, index) MSGPACK_PP_LIST_FIRST(MSGPACK_PP_LIST_REST_N_D(d, index, list))
# endif
#
# endif
