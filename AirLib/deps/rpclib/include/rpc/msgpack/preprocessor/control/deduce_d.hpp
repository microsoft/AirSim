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
# ifndef MSGPACK_PREPROCESSOR_CONTROL_DEDUCE_D_HPP
# define MSGPACK_PREPROCESSOR_CONTROL_DEDUCE_D_HPP
#
# include <rpc/msgpack/preprocessor/control/while.hpp>
# include <rpc/msgpack/preprocessor/detail/auto_rec.hpp>
#
# /* MSGPACK_PP_DEDUCE_D */
#
# define MSGPACK_PP_DEDUCE_D() MSGPACK_PP_AUTO_REC(MSGPACK_PP_WHILE_P, 256)
#
# endif
