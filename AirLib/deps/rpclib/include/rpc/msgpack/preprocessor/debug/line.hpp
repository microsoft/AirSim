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
# ifndef MSGPACK_PREPROCESSOR_DEBUG_LINE_HPP
# define MSGPACK_PREPROCESSOR_DEBUG_LINE_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/iteration/iterate.hpp>
# include <rpc/msgpack/preprocessor/stringize.hpp>
#
# /* MSGPACK_PP_LINE */
#
# if MSGPACK_PP_CONFIG_EXTENDED_LINE_INFO
#    define MSGPACK_PP_LINE(line, file) line MSGPACK_PP_CAT(MSGPACK_PP_LINE_, MSGPACK_PP_IS_ITERATING)(file)
#    define MSGPACK_PP_LINE_MSGPACK_PP_IS_ITERATING(file) #file
#    define MSGPACK_PP_LINE_1(file) MSGPACK_PP_STRINGIZE(file MSGPACK_PP_CAT(MSGPACK_PP_LINE_I_, MSGPACK_PP_ITERATION_DEPTH())())
#    define MSGPACK_PP_LINE_I_1() [MSGPACK_PP_FRAME_ITERATION(1)]
#    define MSGPACK_PP_LINE_I_2() MSGPACK_PP_LINE_I_1()[MSGPACK_PP_FRAME_ITERATION(2)]
#    define MSGPACK_PP_LINE_I_3() MSGPACK_PP_LINE_I_2()[MSGPACK_PP_FRAME_ITERATION(3)]
#    define MSGPACK_PP_LINE_I_4() MSGPACK_PP_LINE_I_3()[MSGPACK_PP_FRAME_ITERATION(4)]
#    define MSGPACK_PP_LINE_I_5() MSGPACK_PP_LINE_I_4()[MSGPACK_PP_FRAME_ITERATION(5)]
# else
#    define MSGPACK_PP_LINE(line, file) line __FILE__
# endif
#
# endif
