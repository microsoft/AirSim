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
# ifndef MSGPACK_PREPROCESSOR_ITERATION_ITERATE_HPP
# define MSGPACK_PREPROCESSOR_ITERATION_ITERATE_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/arithmetic/inc.hpp>
# include <rpc/msgpack/preprocessor/array/elem.hpp>
# include <rpc/msgpack/preprocessor/array/size.hpp>
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/slot/slot.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
#
# /* MSGPACK_PP_ITERATION_DEPTH */
#
# define MSGPACK_PP_ITERATION_DEPTH() 0
#
# /* MSGPACK_PP_ITERATION */
#
# define MSGPACK_PP_ITERATION() MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_, MSGPACK_PP_ITERATION_DEPTH())
#
# /* MSGPACK_PP_ITERATION_START && MSGPACK_PP_ITERATION_FINISH */
#
# define MSGPACK_PP_ITERATION_START() MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_START_, MSGPACK_PP_ITERATION_DEPTH())
# define MSGPACK_PP_ITERATION_FINISH() MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_FINISH_, MSGPACK_PP_ITERATION_DEPTH())
#
# /* MSGPACK_PP_ITERATION_FLAGS */
#
# define MSGPACK_PP_ITERATION_FLAGS() (MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_FLAGS_, MSGPACK_PP_ITERATION_DEPTH())())
#
# /* MSGPACK_PP_FRAME_ITERATION */
#
# define MSGPACK_PP_FRAME_ITERATION(i) MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_, i)
#
# /* MSGPACK_PP_FRAME_START && MSGPACK_PP_FRAME_FINISH */
#
# define MSGPACK_PP_FRAME_START(i) MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_START_, i)
# define MSGPACK_PP_FRAME_FINISH(i) MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_FINISH_, i)
#
# /* MSGPACK_PP_FRAME_FLAGS */
#
# define MSGPACK_PP_FRAME_FLAGS(i) (MSGPACK_PP_CAT(MSGPACK_PP_ITERATION_FLAGS_, i)())
#
# /* MSGPACK_PP_RELATIVE_ITERATION */
#
# define MSGPACK_PP_RELATIVE_ITERATION(i) MSGPACK_PP_CAT(MSGPACK_PP_RELATIVE_, i)(MSGPACK_PP_ITERATION_)
#
# define MSGPACK_PP_RELATIVE_0(m) MSGPACK_PP_CAT(m, MSGPACK_PP_ITERATION_DEPTH())
# define MSGPACK_PP_RELATIVE_1(m) MSGPACK_PP_CAT(m, MSGPACK_PP_DEC(MSGPACK_PP_ITERATION_DEPTH()))
# define MSGPACK_PP_RELATIVE_2(m) MSGPACK_PP_CAT(m, MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_ITERATION_DEPTH())))
# define MSGPACK_PP_RELATIVE_3(m) MSGPACK_PP_CAT(m, MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_ITERATION_DEPTH()))))
# define MSGPACK_PP_RELATIVE_4(m) MSGPACK_PP_CAT(m, MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_DEC(MSGPACK_PP_ITERATION_DEPTH())))))
#
# /* MSGPACK_PP_RELATIVE_START && MSGPACK_PP_RELATIVE_FINISH */
#
# define MSGPACK_PP_RELATIVE_START(i) MSGPACK_PP_CAT(MSGPACK_PP_RELATIVE_, i)(MSGPACK_PP_ITERATION_START_)
# define MSGPACK_PP_RELATIVE_FINISH(i) MSGPACK_PP_CAT(MSGPACK_PP_RELATIVE_, i)(MSGPACK_PP_ITERATION_FINISH_)
#
# /* MSGPACK_PP_RELATIVE_FLAGS */
#
# define MSGPACK_PP_RELATIVE_FLAGS(i) (MSGPACK_PP_CAT(MSGPACK_PP_RELATIVE_, i)(MSGPACK_PP_ITERATION_FLAGS_)())
#
# /* MSGPACK_PP_ITERATE */
#
# define MSGPACK_PP_ITERATE() MSGPACK_PP_CAT(MSGPACK_PP_ITERATE_, MSGPACK_PP_INC(MSGPACK_PP_ITERATION_DEPTH()))
#
# define MSGPACK_PP_ITERATE_1 <msgpack/preprocessor/iteration/detail/iter/forward1.hpp>
# define MSGPACK_PP_ITERATE_2 <msgpack/preprocessor/iteration/detail/iter/forward2.hpp>
# define MSGPACK_PP_ITERATE_3 <msgpack/preprocessor/iteration/detail/iter/forward3.hpp>
# define MSGPACK_PP_ITERATE_4 <msgpack/preprocessor/iteration/detail/iter/forward4.hpp>
# define MSGPACK_PP_ITERATE_5 <msgpack/preprocessor/iteration/detail/iter/forward5.hpp>
#
# endif
