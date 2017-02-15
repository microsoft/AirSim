# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  *
#  * See http://www.boost.org for most recent version.
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# ifndef MSGPACK_PREPROCESSOR_LIST_ADT_HPP
# define MSGPACK_PREPROCESSOR_LIST_ADT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/detail/is_binary.hpp>
# include <rpc/msgpack/preprocessor/logical/compl.hpp>
# include <rpc/msgpack/preprocessor/tuple/eat.hpp>
#
# /* MSGPACK_PP_LIST_CONS */
#
# define MSGPACK_PP_LIST_CONS(head, tail) (head, tail)
#
# /* MSGPACK_PP_LIST_NIL */
#
# define MSGPACK_PP_LIST_NIL MSGPACK_PP_NIL
#
# /* MSGPACK_PP_LIST_FIRST */
#
# define MSGPACK_PP_LIST_FIRST(list) MSGPACK_PP_LIST_FIRST_D(list)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_LIST_FIRST_D(list) MSGPACK_PP_LIST_FIRST_I list
# else
#    define MSGPACK_PP_LIST_FIRST_D(list) MSGPACK_PP_LIST_FIRST_I ## list
# endif
#
# define MSGPACK_PP_LIST_FIRST_I(head, tail) head
#
# /* MSGPACK_PP_LIST_REST */
#
# define MSGPACK_PP_LIST_REST(list) MSGPACK_PP_LIST_REST_D(list)
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_LIST_REST_D(list) MSGPACK_PP_LIST_REST_I list
# else
#    define MSGPACK_PP_LIST_REST_D(list) MSGPACK_PP_LIST_REST_I ## list
# endif
#
# define MSGPACK_PP_LIST_REST_I(head, tail) tail
#
# /* MSGPACK_PP_LIST_IS_CONS */
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_BCC()
#    define MSGPACK_PP_LIST_IS_CONS(list) MSGPACK_PP_LIST_IS_CONS_D(list)
#    define MSGPACK_PP_LIST_IS_CONS_D(list) MSGPACK_PP_LIST_IS_CONS_ ## list
#    define MSGPACK_PP_LIST_IS_CONS_(head, tail) 1
#    define MSGPACK_PP_LIST_IS_CONS_MSGPACK_PP_NIL 0
# else
#    define MSGPACK_PP_LIST_IS_CONS(list) MSGPACK_PP_IS_BINARY(list)
# endif
#
# /* MSGPACK_PP_LIST_IS_NIL */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_BCC()
#    define MSGPACK_PP_LIST_IS_NIL(list) MSGPACK_PP_COMPL(MSGPACK_PP_IS_BINARY(list))
# else
#    define MSGPACK_PP_LIST_IS_NIL(list) MSGPACK_PP_COMPL(MSGPACK_PP_LIST_IS_CONS(list))
# endif
#
# endif
