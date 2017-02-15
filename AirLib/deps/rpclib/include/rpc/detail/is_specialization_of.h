#pragma once

#ifndef IS_SPECIALIZATION_OF_H_OPZTARVG
#define IS_SPECIALIZATION_OF_H_OPZTARVG

#include "rpc/detail/bool.h"

namespace rpc {
namespace detail {

template <template <typename...> class Templ, typename T>
struct is_specialization_of : false_ {};

template <template <typename...> class Templ, typename... T>
struct is_specialization_of<Templ, Templ<T...>> : true_ {};

}
} /* rpc  */

#endif /* end of include guard: IS_SPECIALIZATION_OF_H_OPZTARVG */
