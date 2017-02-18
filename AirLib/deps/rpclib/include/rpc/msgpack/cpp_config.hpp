//
// MessagePack for C++ C++03/C++11 Adaptation
//
// Copyright (C) 2013 KONDO Takatoshi
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
#ifndef MSGPACK_CPP_CONFIG_HPP
#define MSGPACK_CPP_CONFIG_HPP

#include "rpc/msgpack/versioning.hpp"

#if !defined(MSGPACK_USE_CPP03)
# if defined(_MSC_VER)
#  if _MSC_VER < 1900
#    define MSGPACK_USE_CPP03
#  endif
# elif (__cplusplus < 201103L)
#  define MSGPACK_USE_CPP03
# endif
#endif // MSGPACK_USE_CPP03



#if defined __cplusplus
#if __cplusplus < 201103L

#if !defined(nullptr)
#  if _MSC_VER < 1600
#    define nullptr (0)
#  endif
#endif

#include <memory>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

template <typename T>
struct unique_ptr : std::auto_ptr<T> {
    explicit unique_ptr(T* p = 0) throw() : std::auto_ptr<T>(p) {}
    unique_ptr(unique_ptr& a) throw() : std::auto_ptr<T>(a) {}
    template<class Y>
    unique_ptr (unique_ptr<Y>& a) throw() : std::auto_ptr<T>(a) {}
};

template <typename T>
T& move(T& t)
{
    return t;
}

template <typename T>
T const& move(T const& t)
{
    return t;
}

template <bool P, typename T = void>
struct enable_if {
    typedef T type;
};

template <typename T>
struct enable_if<false, T> {
};

template<typename T, T val>
struct integral_constant {
    static T const value = val;
    typedef T value_type;
    typedef integral_constant<T, val> type;
};

typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;

template<class T, class U>
struct is_same : false_type {};

template<class T>
struct is_same<T, T> : true_type {};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack


#else  // __cplusplus < 201103L

#include <memory>
#include <tuple>

namespace clmdep_msgpack {
/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

    // unique_ptr
    using std::unique_ptr;
    // using std::make_unique; // since C++14
    using std::hash;

    // utility
    using std::move;
    using std::swap;
    using std::enable_if;
    using std::is_same;

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond
}  // namespace clmdep_msgpack


#endif // __cplusplus < 201103L

#endif // __cplusplus

#endif /* msgpack/cpp_config.hpp */
