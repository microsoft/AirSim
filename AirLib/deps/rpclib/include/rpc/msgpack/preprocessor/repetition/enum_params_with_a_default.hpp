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
# ifndef MSGPACK_PREPROCESSOR_REPETITION_ENUM_PARAMS_WITH_A_DEFAULT_HPP
# define MSGPACK_PREPROCESSOR_REPETITION_ENUM_PARAMS_WITH_A_DEFAULT_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/facilities/intercept.hpp>
# include <rpc/msgpack/preprocessor/repetition/enum_binary_params.hpp>
#
# /* MSGPACK_PP_ENUM_PARAMS_WITH_A_DEFAULT */
#
# define MSGPACK_PP_ENUM_PARAMS_WITH_A_DEFAULT(count, param, def) MSGPACK_PP_ENUM_BINARY_PARAMS(count, param, = def MSGPACK_PP_INTERCEPT)
#
# endif
