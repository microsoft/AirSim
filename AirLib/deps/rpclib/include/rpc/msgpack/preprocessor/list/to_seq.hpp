# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2011.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* Revised by Paul Mensonides (2011) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_LIST_TO_SEQ_HPP
# define MSGPACK_PREPROCESSOR_LIST_TO_SEQ_HPP
#
# include <rpc/msgpack/preprocessor/list/for_each.hpp>
#
# /* MSGPACK_PP_LIST_TO_SEQ */
#
# define MSGPACK_PP_LIST_TO_SEQ(list) \
    MSGPACK_PP_LIST_FOR_EACH(MSGPACK_PP_LIST_TO_SEQ_MACRO, ~, list) \
    /**/
# define MSGPACK_PP_LIST_TO_SEQ_MACRO(r, data, elem) (elem)
#
# /* MSGPACK_PP_LIST_TO_SEQ_R */
#
# define MSGPACK_PP_LIST_TO_SEQ_R(r, list) \
    MSGPACK_PP_LIST_FOR_EACH_R(r, MSGPACK_PP_LIST_TO_SEQ_MACRO, ~, list) \
    /**/
#
# endif /* MSGPACK_PREPROCESSOR_LIST_TO_SEQ_HPP */
