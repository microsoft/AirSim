# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2003.
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_HPP
# define MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_HPP
#
# include <rpc/msgpack/preprocessor/config/config.hpp>
#
# if MSGPACK_PP_VARIADICS
#
# include <rpc/msgpack/preprocessor/facilities/is_empty_variadic.hpp>
#
# else
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC() && ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
# include <rpc/msgpack/preprocessor/tuple/elem.hpp>
# include <rpc/msgpack/preprocessor/facilities/identity.hpp>
# else
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/detail/split.hpp>
# endif
#
# /* MSGPACK_PP_IS_EMPTY */
#
# if ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC() && ~MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MWCC()
#    define MSGPACK_PP_IS_EMPTY(x) MSGPACK_PP_IS_EMPTY_I(x MSGPACK_PP_IS_EMPTY_HELPER)
#    define MSGPACK_PP_IS_EMPTY_I(contents) MSGPACK_PP_TUPLE_ELEM(2, 1, (MSGPACK_PP_IS_EMPTY_DEF_ ## contents()))
#    define MSGPACK_PP_IS_EMPTY_DEF_MSGPACK_PP_IS_EMPTY_HELPER 1, MSGPACK_PP_IDENTITY(1)
#    define MSGPACK_PP_IS_EMPTY_HELPER() , 0
# else
#    if MSGPACK_PP_CONFIG_FLAGS() & MSGPACK_PP_CONFIG_MSVC()
#        define MSGPACK_PP_IS_EMPTY(x) MSGPACK_PP_IS_EMPTY_I(MSGPACK_PP_IS_EMPTY_HELPER x ())
#        define MSGPACK_PP_IS_EMPTY_I(test) MSGPACK_PP_IS_EMPTY_II(MSGPACK_PP_SPLIT(0, MSGPACK_PP_CAT(MSGPACK_PP_IS_EMPTY_DEF_, test)))
#        define MSGPACK_PP_IS_EMPTY_II(id) id
#    else
#        define MSGPACK_PP_IS_EMPTY(x) MSGPACK_PP_IS_EMPTY_I((MSGPACK_PP_IS_EMPTY_HELPER x ()))
#        define MSGPACK_PP_IS_EMPTY_I(par) MSGPACK_PP_IS_EMPTY_II ## par
#        define MSGPACK_PP_IS_EMPTY_II(test) MSGPACK_PP_SPLIT(0, MSGPACK_PP_CAT(MSGPACK_PP_IS_EMPTY_DEF_, test))
#    endif
#    define MSGPACK_PP_IS_EMPTY_HELPER() 1
#    define MSGPACK_PP_IS_EMPTY_DEF_1 1, MSGPACK_PP_NIL
#    define MSGPACK_PP_IS_EMPTY_DEF_MSGPACK_PP_IS_EMPTY_HELPER 0, MSGPACK_PP_NIL
# endif
#
# endif /* MSGPACK_PP_VARIADICS */
#
# endif /* MSGPACK_PREPROCESSOR_FACILITIES_IS_EMPTY_HPP */
