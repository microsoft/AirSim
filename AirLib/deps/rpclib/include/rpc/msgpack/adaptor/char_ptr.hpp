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
#ifndef MSGPACK_TYPE_CHAR_PTR_HPP
#define MSGPACK_TYPE_CHAR_PTR_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/object_fwd.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <cstring>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <>
struct pack<const char*> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.pack_str(size);
        o.pack_str_body(v, size);
        return o;
    }
};

template <>
struct object_with_zone<const char*> {
    void operator()(clmdep_msgpack::object::with_zone& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size));
        o.via.str.ptr = ptr;
        o.via.str.size = size;
        std::memcpy(ptr, v, size);
    }
};

template <>
struct object<const char*> {
    void operator()(clmdep_msgpack::object& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v;
        o.via.str.size = size;
    }
};


template <>
struct pack<char*> {
    template <typename Stream>
    packer<Stream>& operator()(packer<Stream>& o, char* v) const {
        return o << static_cast<const char*>(v);
    }
};

template <>
struct object_with_zone<char*> {
    void operator()(clmdep_msgpack::object::with_zone& o, char* v) const {
        o << static_cast<const char*>(v);
    }
};

template <>
struct object<char*> {
    void operator()(clmdep_msgpack::object& o, char* v) const {
        o << static_cast<const char*>(v);
    }
};

template <std::size_t N>
struct pack<char[N]> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.pack_str(size);
        o.pack_str_body(v, size);
        return o;
    }
};

template <std::size_t N>
struct object_with_zone<char[N]> {
    void operator()(clmdep_msgpack::object::with_zone& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size));
        o.via.str.ptr = ptr;
        o.via.str.size = size;
        std::memcpy(ptr, v, size);
    }
};

template <std::size_t N>
struct object<char[N]> {
    void operator()(clmdep_msgpack::object& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v;
        o.via.str.size = size;
    }
};

template <std::size_t N>
struct pack<const char[N]> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.pack_str(size);
        o.pack_str_body(v, size);
        return o;
    }
};

template <std::size_t N>
struct object_with_zone<const char[N]> {
    void operator()(clmdep_msgpack::object::with_zone& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size));
        o.via.str.ptr = ptr;
        o.via.str.size = size;
        std::memcpy(ptr, v, size);
    }
};

template <std::size_t N>
struct object<const char[N]> {
    void operator()(clmdep_msgpack::object& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v;
        o.via.str.size = size;
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_CHAR_PTR_HPP
