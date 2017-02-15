//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 KONDO Takatoshi
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
//

#ifndef MSGPACK_META_HPP
#define MSGPACK_META_HPP

#if !defined(MSGPACK_USE_CPP03)

#include <type_traits>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace detail {
template<bool...> struct bool_pack;

template<bool...values> struct all_of_imp
    : std::is_same<bool_pack<values..., true>, bool_pack<true, values...>>{};

} // namespace detail

template<template <class> class T, class... U>
using all_of = detail::all_of_imp<T<U>::value...>;

template<std::size_t... Is> struct seq {};

template<std::size_t N, std::size_t... Is>
struct gen_seq : gen_seq<N-1, N-1, Is...> {};

template<std::size_t... Is>
struct gen_seq<0, Is...> : seq<Is...> {};

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // !defined(MSGPACK_USE_CPP03)

#endif // MSGPACK_META_HPP
