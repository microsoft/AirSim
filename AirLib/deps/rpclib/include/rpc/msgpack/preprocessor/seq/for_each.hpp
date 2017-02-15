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
# ifndef MSGPACK_PREPROCESSOR_SEQ_FOR_EACH_HPP
# define MSGPACK_PREPROCESSOR_SEQ_FOR_EACH_HPP
#
# include <rpc/msgpack/preprocessor/arithmetic/dec.hpp>
# include <rpc/msgpack/preprocessor/config/config.hpp>
# include <rpc/msgpack/preprocessor/repetition/for.hpp>
# include <rpc/msgpack/preprocessor/seq/seq.hpp>
# include <rpc/msgpack/preprocessor/seq/size.hpp>
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/tuple/rem.hpp>
#
# /* MSGPACK_PP_SEQ_FOR_EACH */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_FOR_EACH(macro, data, seq) MSGPACK_PP_FOR((macro, data, seq (nil)), MSGPACK_PP_SEQ_FOR_EACH_P, MSGPACK_PP_SEQ_FOR_EACH_O, MSGPACK_PP_SEQ_FOR_EACH_M)
# else
#    define MSGPACK_PP_SEQ_FOR_EACH(macro, data, seq) MSGPACK_PP_SEQ_FOR_EACH_D(macro, data, seq)
#    define MSGPACK_PP_SEQ_FOR_EACH_D(macro, data, seq) MSGPACK_PP_FOR((macro, data, seq (nil)), MSGPACK_PP_SEQ_FOR_EACH_P, MSGPACK_PP_SEQ_FOR_EACH_O, MSGPACK_PP_SEQ_FOR_EACH_M)
# endif
#
# define MSGPACK_PP_SEQ_FOR_EACH_P(r, x) MSGPACK_PP_DEC(MSGPACK_PP_SEQ_SIZE(MSGPACK_PP_TUPLE_ELEM(3, 2, x)))
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_SEQ_FOR_EACH_O(r, x) MSGPACK_PP_SEQ_FOR_EACH_O_I x
# else
#    define MSGPACK_PP_SEQ_FOR_EACH_O(r, x) MSGPACK_PP_SEQ_FOR_EACH_O_I(MSGPACK_PP_TUPLE_ELEM(3, 0, x), MSGPACK_PP_TUPLE_ELEM(3, 1, x), MSGPACK_PP_TUPLE_ELEM(3, 2, x))
# endif
#
# define MSGPACK_PP_SEQ_FOR_EACH_O_I(macro, data, seq) (macro, data, MSGPACK_PP_SEQ_TAIL(seq))
#
# if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_STRICT()
#    define MSGPACK_PP_SEQ_FOR_EACH_M(r, x) MSGPACK_PP_SEQ_FOR_EACH_M_IM(r, MSGPACK_PP_TUPLE_REM_3 x)
#    define MSGPACK_PP_SEQ_FOR_EACH_M_IM(r, im) MSGPACK_PP_SEQ_FOR_EACH_M_I(r, im)
# else
#    define MSGPACK_PP_SEQ_FOR_EACH_M(r, x) MSGPACK_PP_SEQ_FOR_EACH_M_I(r, MSGPACK_PP_TUPLE_ELEM(3, 0, x), MSGPACK_PP_TUPLE_ELEM(3, 1, x), MSGPACK_PP_TUPLE_ELEM(3, 2, x))
# endif
#
# define MSGPACK_PP_SEQ_FOR_EACH_M_I(r, macro, data, seq) macro(r, data, MSGPACK_PP_SEQ_HEAD(seq))
#
# /* MSGPACK_PP_SEQ_FOR_EACH_R */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_EDG()
#    define MSGPACK_PP_SEQ_FOR_EACH_R(r, macro, data, seq) MSGPACK_PP_FOR_ ## r((macro, data, seq (nil)), MSGPACK_PP_SEQ_FOR_EACH_P, MSGPACK_PP_SEQ_FOR_EACH_O, MSGPACK_PP_SEQ_FOR_EACH_M)
# else
#    define MSGPACK_PP_SEQ_FOR_EACH_R(r, macro, data, seq) MSGPACK_PP_SEQ_FOR_EACH_R_I(r, macro, data, seq)
#    define MSGPACK_PP_SEQ_FOR_EACH_R_I(r, macro, data, seq) MSGPACK_PP_FOR_ ## r((macro, data, seq (nil)), MSGPACK_PP_SEQ_FOR_EACH_P, MSGPACK_PP_SEQ_FOR_EACH_O, MSGPACK_PP_SEQ_FOR_EACH_M)
# endif
#
# endif
