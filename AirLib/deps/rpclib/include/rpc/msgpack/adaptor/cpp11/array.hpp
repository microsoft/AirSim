//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2014-2015 KONDO Takatoshi
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

#ifndef MSGPACK_CPP11_ARRAY_HPP
#define MSGPACK_CPP11_ARRAY_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"
#include "rpc/msgpack/meta.hpp"

#include <array>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

namespace detail {

namespace array {

template<typename T, std::size_t N1, std::size_t... I1, std::size_t N2, std::size_t... I2>
inline std::array<T, N1+N2> concat(
    std::array<T, N1>&& a1,
    std::array<T, N2>&& a2,
    clmdep_msgpack::seq<I1...>,
    clmdep_msgpack::seq<I2...>) {
    return {{ std::move(a1[I1])..., std::move(a2[I2])... }};
}

template<typename T, std::size_t N1, std::size_t N2>
inline std::array<T, N1+N2> concat(std::array<T, N1>&& a1, std::array<T, N2>&& a2) {
    return concat(std::move(a1), std::move(a2), clmdep_msgpack::gen_seq<N1>(), clmdep_msgpack::gen_seq<N2>());
}

template <typename T, std::size_t N>
struct as_impl {
    static std::array<T, N> as(clmdep_msgpack::object const& o) {
        clmdep_msgpack::object* p = o.via.array.ptr + N - 1;
        return concat(as_impl<T, N-1>::as(o), std::array<T, 1>{{p->as<T>()}});
    }
};

template <typename T>
struct as_impl<T, 1> {
    static std::array<T, 1> as(clmdep_msgpack::object const& o) {
        clmdep_msgpack::object* p = o.via.array.ptr;
        return std::array<T, 1>{{p->as<T>()}};
    }
};

template <typename T>
struct as_impl<T, 0> {
    static std::array<T, 0> as(clmdep_msgpack::object const&) {
        return std::array<T, 0>();
    }
};

} // namespace array

} // namespace detail

template <typename T, std::size_t N>
struct as<std::array<T, N>, typename std::enable_if<clmdep_msgpack::has_as<T>::value>::type> {
    std::array<T, N> operator()(clmdep_msgpack::object const& o) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if(o.via.array.size != N) { throw clmdep_msgpack::type_error(); }
        return detail::array::as_impl<T, N>::as(o);
    }
};

template <typename T, std::size_t N>
struct convert<std::array<T, N>> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::array<T, N>& v) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if(o.via.array.size != N) { throw clmdep_msgpack::type_error(); }
        if(o.via.array.size > 0) {
            clmdep_msgpack::object* p = o.via.array.ptr;
            clmdep_msgpack::object* const pend = o.via.array.ptr + o.via.array.size;
            T* it = &v[0];
            do {
                p->convert(*it);
                ++p;
                ++it;
            } while(p < pend);
        }
        return o;
    }
};

template <typename T, std::size_t N>
struct pack<std::array<T, N>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::array<T, N>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_array(size);
        for(auto const& e : v) o.pack(e);
        return o;
    }
};

template <typename T, std::size_t N>
struct object_with_zone<std::array<T, N>> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::array<T, N>& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        if(v.empty()) {
            o.via.array.ptr = nullptr;
            o.via.array.size = 0;
        } else {
            uint32_t size = checked_get_container_size(v.size());
            clmdep_msgpack::object* p = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size));
            o.via.array.size = size;
            o.via.array.ptr = p;
            for (auto const& e : v) *p++ = clmdep_msgpack::object(e, o.zone);
        }
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_CPP11_ARRAY_HPP
