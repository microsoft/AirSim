//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2015 FURUHASHI Sadayuki and KONDO Takatoshi
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
#ifndef MSGPACK_TYPE_VECTOR_HPP
#define MSGPACK_TYPE_VECTOR_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <vector>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

#if !defined(MSGPACK_USE_CPP03)

template <typename T, typename Alloc>
struct as<std::vector<T, Alloc>, typename std::enable_if<clmdep_msgpack::has_as<T>::value>::type> {
    std::vector<T, Alloc> operator()(const clmdep_msgpack::object& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        std::vector<T, Alloc> v;
        v.reserve(o.via.array.size);
        if (o.via.array.size > 0) {
            clmdep_msgpack::object* p = o.via.array.ptr;
            clmdep_msgpack::object* const pend = o.via.array.ptr + o.via.array.size;
            do {
                v.push_back(p->as<T>());
                ++p;
            } while (p < pend);
        }
        return v;
    }
};

#endif // !defined(MSGPACK_USE_CPP03)

template <typename T, typename Alloc>
struct convert<std::vector<T, Alloc> > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::vector<T, Alloc>& v) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        v.resize(o.via.array.size);
        if (o.via.array.size > 0) {
            clmdep_msgpack::object* p = o.via.array.ptr;
            clmdep_msgpack::object* const pend = o.via.array.ptr + o.via.array.size;
            typename std::vector<T, Alloc>::iterator it = v.begin();
            do {
                p->convert(*it);
                ++p;
                ++it;
            } while(p < pend);
        }
        return o;
    }
};

template <typename T, typename Alloc>
struct pack<std::vector<T, Alloc> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::vector<T, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_array(size);
        for (typename std::vector<T, Alloc>::const_iterator it(v.begin()), it_end(v.end());
            it != it_end; ++it) {
            o.pack(*it);
        }
        return o;
    }
};

template <typename T, typename Alloc>
struct object_with_zone<std::vector<T, Alloc> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::vector<T, Alloc>& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        if (v.empty()) {
            o.via.array.ptr = nullptr;
            o.via.array.size = 0;
        }
        else {
            uint32_t size = checked_get_container_size(v.size());
            clmdep_msgpack::object* p = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size));
            clmdep_msgpack::object* const pend = p + size;
            o.via.array.ptr = p;
            o.via.array.size = size;
            typename std::vector<T, Alloc>::const_iterator it(v.begin());
            do {
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) && !defined(__clang__)
                *p = clmdep_msgpack::object(*it, o.zone);
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)) && !defined(__clang__)
                ++p;
                ++it;
            } while(p < pend);
        }
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_VECTOR_HPP
