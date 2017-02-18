//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2009 FURUHASHI Sadayuki
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
#ifndef MSGPACK_TYPE_RAW_HPP
#define MSGPACK_TYPE_RAW_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include <cstring>
#include <string>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {

struct raw_ref {
    raw_ref() : size(0), ptr(nullptr) {}
    raw_ref(const char* p, uint32_t s) : size(s), ptr(p) {}

    uint32_t size;
    const char* ptr;

    std::string str() const { return std::string(ptr, size); }

    bool operator== (const raw_ref& x) const
    {
        return size == x.size && std::memcmp(ptr, x.ptr, size) == 0;
    }

    bool operator!= (const raw_ref& x) const
    {
        return !(*this == x);
    }

    bool operator< (const raw_ref& x) const
    {
        if(size == x.size) { return std::memcmp(ptr, x.ptr, size) < 0; }
        else { return size < x.size; }
    }

    bool operator> (const raw_ref& x) const
    {
        if(size == x.size) { return std::memcmp(ptr, x.ptr, size) > 0; }
        else { return size > x.size; }
    }
};

} // namespace type

namespace adaptor {

template <>
struct convert<clmdep_msgpack::type::raw_ref> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, clmdep_msgpack::type::raw_ref& v) const {
        if(o.type != clmdep_msgpack::type::BIN) { throw clmdep_msgpack::type_error(); }
        v.ptr  = o.via.bin.ptr;
        v.size = o.via.bin.size;
        return o;
    }
};

template <>
struct pack<clmdep_msgpack::type::raw_ref> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::type::raw_ref& v) const {
        o.pack_bin(v.size);
        o.pack_bin_body(v.ptr, v.size);
        return o;
    }
};

template <>
struct object<clmdep_msgpack::type::raw_ref> {
    void operator()(clmdep_msgpack::object& o, const clmdep_msgpack::type::raw_ref& v) const {
        o.type = clmdep_msgpack::type::BIN;
        o.via.bin.ptr = v.ptr;
        o.via.bin.size = v.size;
    }
};

template <>
struct object_with_zone<clmdep_msgpack::type::raw_ref> {
    void operator()(clmdep_msgpack::object::with_zone& o, const clmdep_msgpack::type::raw_ref& v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_RAW_HPP
