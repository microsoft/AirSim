# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
#ifndef MSGPACK_PREPROCESSOR_REMOVE_PARENS_HPP
#define MSGPACK_PREPROCESSOR_REMOVE_PARENS_HPP

#include <rpc/msgpack/preprocessor/config/config.hpp>

#if MSGPACK_PP_VARIADICS

#include <rpc/msgpack/preprocessor/control/iif.hpp>
#include <rpc/msgpack/preprocessor/facilities/identity.hpp>
#include <rpc/msgpack/preprocessor/punctuation/is_begin_parens.hpp>
#include <rpc/msgpack/preprocessor/tuple/enum.hpp>

#define MSGPACK_PP_REMOVE_PARENS(param) \
    MSGPACK_PP_IIF \
      ( \
      MSGPACK_PP_IS_BEGIN_PARENS(param), \
      MSGPACK_PP_REMOVE_PARENS_DO, \
      MSGPACK_PP_IDENTITY \
      ) \
    (param)() \
/**/

#define MSGPACK_PP_REMOVE_PARENS_DO(param) \
  MSGPACK_PP_IDENTITY(MSGPACK_PP_TUPLE_ENUM(param)) \
/**/

#endif /* MSGPACK_PP_VARIADICS */
#endif /* MSGPACK_PREPROCESSOR_REMOVE_PARENS_HPP */
