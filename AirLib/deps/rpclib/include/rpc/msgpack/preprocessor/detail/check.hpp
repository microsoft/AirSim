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
# ifndef MSGPACK_PREPROCESSOR_DETAIL_CHECK_HPP
# define MSGPACK_PREPROCESSOR_DETAIL_CHECK_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# /* MSGPACK_PP_CHECK */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_CHECK(x, type) MSGPACK_PP_CHECK_D(x, type)
# else
#    define MSGPACK_PP_CHECK(x, type) MSGPACK_PP_CHECK_OO((x, type))
#    define MSGPACK_PP_CHECK_OO(par) MSGPACK_PP_CHECK_D ## par
# endif
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC() && ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_DMC()
#    define MSGPACK_PP_CHECK_D(x, type) MSGPACK_PP_CHECK_1(MSGPACK_PP_CAT(MSGPACK_PP_CHECK_RESULT_, type x))
#    define MSGPACK_PP_CHECK_1(chk) MSGPACK_PP_CHECK_2(chk)
#    define MSGPACK_PP_CHECK_2(res, _) res
# elif MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#    define MSGPACK_PP_CHECK_D(x, type) MSGPACK_PP_CHECK_1(type x)
#    define MSGPACK_PP_CHECK_1(chk) MSGPACK_PP_CHECK_2(chk)
#    define MSGPACK_PP_CHECK_2(chk) MSGPACK_PP_CHECK_3((MSGPACK_PP_CHECK_RESULT_ ## chk))
#    define MSGPACK_PP_CHECK_3(im) MSGPACK_PP_CHECK_5(MSGPACK_PP_CHECK_4 im)
#    define MSGPACK_PP_CHECK_4(res, _) res
#    define MSGPACK_PP_CHECK_5(res) res
# else /* DMC */
#    define MSGPACK_PP_CHECK_D(x, type) MSGPACK_PP_CHECK_OO((type x))
#    define MSGPACK_PP_CHECK_OO(par) MSGPACK_PP_CHECK_0 ## par
#    define MSGPACK_PP_CHECK_0(chk) MSGPACK_PP_CHECK_1(MSGPACK_PP_CAT(MSGPACK_PP_CHECK_RESULT_, chk))
#    define MSGPACK_PP_CHECK_1(chk) MSGPACK_PP_CHECK_2(chk)
#    define MSGPACK_PP_CHECK_2(res, _) res
# endif
#
# define MSGPACK_PP_CHECK_RESULT_1 1, MSGPACK_PP_NIL
#
# endif
