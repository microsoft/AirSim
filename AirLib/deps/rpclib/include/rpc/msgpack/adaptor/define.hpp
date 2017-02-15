//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2014 FURUHASHI Sadayuki and KONDO Takatoshi
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
#ifndef MSGPACK_DEFINE_HPP
#define MSGPACK_DEFINE_HPP

#include "rpc/msgpack/cpp_config.hpp"

#if defined(MSGPACK_USE_CPP03)
#include "detail/cpp03_define_array.hpp"
#include "detail/cpp03_define_map.hpp"
#else  // MSGPACK_USE_CPP03
#include "detail/cpp11_define_array.hpp"
#include "detail/cpp11_define_map.hpp"
#endif // MSGPACK_USE_CPP03

#if defined(MSGPACK_USE_DEFINE_MAP)
#define MSGPACK_DEFINE MSGPACK_DEFINE_MAP
#define MSGPACK_BASE MSGPACK_BASE_MAP
#else  // defined(MSGPACK_USE_DEFINE_MAP)
#define MSGPACK_DEFINE MSGPACK_DEFINE_ARRAY
#define MSGPACK_BASE MSGPACK_BASE_ARRAY
#endif // defined(MSGPACK_USE_DEFINE_MAP)

#endif // MSGPACK_DEFINE_HPP
