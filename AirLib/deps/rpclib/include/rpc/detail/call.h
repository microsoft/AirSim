#pragma once

#ifndef CALL_H_ZXFACADH
#define CALL_H_ZXFACADH

#include <tuple>
#include "rpc/detail/func_tools.h"
#include "rpc/detail/invoke.h"
#include "rpc/detail/is_specialization_of.h"

namespace rpc {
namespace detail {

template <typename Functor, typename... Args, std::size_t... I>
decltype(auto) call_helper(Functor func, std::tuple<Args...> &&params,
                           std::index_sequence<I...>) {
    return func(std::get<I>(params)...);
}

//! \brief Calls a functor with arguments provided as a tuple

template <typename Functor, typename Arg>
decltype(auto) call(Functor f, Arg &&arg) {
    return f(std::forward<Arg>(arg));
}

template <typename Functor, typename... Args>
decltype(auto) call(Functor f, std::tuple<Args...> &args) {
    return call_helper(f, std::forward<std::tuple<Args...>>(args),
                       std::index_sequence_for<Args...>{});
}
}
}

#endif /* end of include guard: CALL_H_ZXFACADH */
