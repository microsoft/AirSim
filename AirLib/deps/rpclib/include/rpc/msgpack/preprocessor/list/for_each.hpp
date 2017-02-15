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
# ifndef MSGPACK_PREPROCESSOR_LIST_FOR_EACH_HPP
# define MSGPACK_PREPROCESSOR_LIST_FOR_EACH_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/list/for_each_i.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_LIST_FOR_EACH */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FOR_EACH(macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I(MSGPACK_PP_LIST_FOR_EACH_O, (macro, data), list)
# else
#    define MSGPACK_PP_LIST_FOR_EACH(macro, data, list) MSGPACK_PP_LIST_FOR_EACH_X(macro, data, list)
#    define MSGPACK_PP_LIST_FOR_EACH_X(macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I(MSGPACK_PP_LIST_FOR_EACH_O, (macro, data), list)
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FOR_EACH_O(r, md, i, elem) MSGPACK_PP_LIST_FOR_EACH_O_D(r, MSGPACK_PP_TUPLE_ELEM(2, 0, md), MSGPACK_PP_TUPLE_ELEM(2, 1, md), elem)
# else
#    define MSGPACK_PP_LIST_FOR_EACH_O(r, md, i, elem) MSGPACK_PP_LIST_FOR_EACH_O_I(r, MSGPACK_PP_TUPLE_REM_2 md, elem)
#    define MSGPACK_PP_LIST_FOR_EACH_O_I(r, im, elem) MSGPACK_PP_LIST_FOR_EACH_O_D(r, im, elem)
# endif
#
# define MSGPACK_PP_LIST_FOR_EACH_O_D(r, m, d, elem) m(r, d, elem)
#
# /* MSGPACK_PP_LIST_FOR_EACH_R */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_LIST_FOR_EACH_R(r, macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I_R(r, MSGPACK_PP_LIST_FOR_EACH_O, (macro, data), list)
# else
#    define MSGPACK_PP_LIST_FOR_EACH_R(r, macro, data, list) MSGPACK_PP_LIST_FOR_EACH_R_X(r, macro, data, list)
#    define MSGPACK_PP_LIST_FOR_EACH_R_X(r, macro, data, list) MSGPACK_PP_LIST_FOR_EACH_I_R(r, MSGPACK_PP_LIST_FOR_EACH_O, (macro, data), list)
# endif
#
# endif
