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
# ifndef MSGPACK_PREPROCESSOR_SLOT_SLOT_HPP
# define MSGPACK_PREPROCESSOR_SLOT_SLOT_HPP
#
# include <rpc/msgpack/preprocessor/cat.hpp>
# include <rpc/msgpack/preprocessor/slot/detail/def.hpp>
#
# /* MSGPACK_PP_ASSIGN_SLOT */
#
# define MSGPACK_PP_ASSIGN_SLOT(i) MSGPACK_PP_CAT(MSGPACK_PP_ASSIGN_SLOT_, i)
#
# define MSGPACK_PP_ASSIGN_SLOT_1 <msgpack/preprocessor/slot/detail/slot1.hpp>
# define MSGPACK_PP_ASSIGN_SLOT_2 <msgpack/preprocessor/slot/detail/slot2.hpp>
# define MSGPACK_PP_ASSIGN_SLOT_3 <msgpack/preprocessor/slot/detail/slot3.hpp>
# define MSGPACK_PP_ASSIGN_SLOT_4 <msgpack/preprocessor/slot/detail/slot4.hpp>
# define MSGPACK_PP_ASSIGN_SLOT_5 <msgpack/preprocessor/slot/detail/slot5.hpp>
#
# /* MSGPACK_PP_SLOT */
#
# define MSGPACK_PP_SLOT(i) MSGPACK_PP_CAT(MSGPACK_PP_SLOT_, i)()
#
# endif
