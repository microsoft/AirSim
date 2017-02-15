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
# ifndef MSGPACK_PREPROCESSOR_DEBUG_ERROR_HPP
# define MSGPACK_PREPROCESSOR_DEBUG_ERROR_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_ERROR */
#
# if MSGPACK_PP_CONFIG_ERRORS
#    define MSGPACK_PP_ERROR(code) MSGPACK_PP_CAT(MSGPACK_PP_ERROR_, code)
# endif
#
# define MSGPACK_PP_ERROR_0x0000 MSGPACK_PP_ERROR(0x0000, MSGPACK_PP_INDEX_OUT_OF_BOUNDS)
# define MSGPACK_PP_ERROR_0x0001 MSGPACK_PP_ERROR(0x0001, MSGPACK_PP_WHILE_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0002 MSGPACK_PP_ERROR(0x0002, MSGPACK_PP_FOR_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0003 MSGPACK_PP_ERROR(0x0003, MSGPACK_PP_REPEAT_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0004 MSGPACK_PP_ERROR(0x0004, MSGPACK_PP_LIST_FOLD_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0005 MSGPACK_PP_ERROR(0x0005, MSGPACK_PP_SEQ_FOLD_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0006 MSGPACK_PP_ERROR(0x0006, MSGPACK_PP_ARITHMETIC_OVERFLOW)
# define MSGPACK_PP_ERROR_0x0007 MSGPACK_PP_ERROR(0x0007, MSGPACK_PP_DIVISION_BY_ZERO)
#
# endif
