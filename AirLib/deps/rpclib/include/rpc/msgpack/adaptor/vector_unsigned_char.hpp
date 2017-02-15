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
#ifndef MSGPACK_TYPE_VECTOR_UNSIGNED_CHAR_HPP
#define MSGPACK_TYPE_VECTOR_UNSIGNED_CHAR_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <vector>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename Alloc>
struct convert<std::vector<unsigned char, Alloc> > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::vector<unsigned char, Alloc>& v) const {
        switch (o.type) {
        case clmdep_msgpack::type::BIN:
            v.resize(o.via.bin.size);
            std::memcpy(&v.front(), o.via.bin.ptr, o.via.bin.size);
            break;
        case clmdep_msgpack::type::STR:
            v.resize(o.via.str.size);
            std::memcpy(&v.front(), o.via.str.ptr, o.via.str.size);
            break;
        default:
            throw clmdep_msgpack::type_error();
            break;
        }
        return o;
    }
};

template <typename Alloc>
struct pack<std::vector<unsigned char, Alloc> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::vector<unsigned char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_bin(size);
        o.pack_bin_body(reinterpret_cast<char const*>(&v.front()), size);

        return o;
    }
};

template <typename Alloc>
struct object<std::vector<unsigned char, Alloc> > {
    void operator()(clmdep_msgpack::object& o, const std::vector<unsigned char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::BIN;
        o.via.bin.ptr = reinterpret_cast<char const*>(&v.front());
        o.via.bin.size = size;
    }
};

template <typename Alloc>
struct object_with_zone<std::vector<unsigned char, Alloc> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::vector<unsigned char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::BIN;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size));
        o.via.bin.ptr = ptr;
        o.via.bin.size = size;
        std::memcpy(ptr, &v.front(), size);
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_VECTOR_UNSIGNED_CHAR_HPP
