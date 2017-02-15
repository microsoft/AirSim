# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2003.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_OR_1_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_OR_1_HPP
#
# include <rpc/msgpack/preprocessor/control/iif.hpp>
# include <rpc/msgpack/preprocessor/facilities/empty.hpp>
# include <rpc/msgpack/preprocessor/facilities/identity.hpp>
# include <rpc/msgpack/preprocessor/facilities/is_1.hpp>
# include <rpc/msgpack/preprocessor/facilities/is_empty.hpp>
#
# /* MSGPACK_PP_IS_EMPTY_OR_1 */
#
# define MSGPACK_PP_IS_EMPTY_OR_1(x) \
    MSGPACK_PP_IIF( \
        MSGPACK_PP_IS_EMPTY(x MSGPACK_PP_EMPTY()), \
        MSGPACK_PP_IDENTITY(1), \
        MSGPACK_PP_IS_1 \
    )(x) \
    /**/
#
# endif
