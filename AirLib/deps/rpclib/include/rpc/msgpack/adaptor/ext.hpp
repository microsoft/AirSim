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
#ifndef MSGPACK_TYPE_EXT_HPP
#define MSGPACK_TYPE_EXT_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include <cstring>
#include <string>
#include <cassert>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {
class ext_ref;

class ext {
public:
    ext() : m_data(1, 0) {}
    ext(int8_t t, const char* p, uint32_t s) {
        detail::check_container_size_for_ext<sizeof(std::size_t)>(s);
        m_data.reserve(static_cast<std::size_t>(s) + 1);
        m_data.push_back(static_cast<char>(t));
        m_data.insert(m_data.end(), p, p + s);
    }
    ext(int8_t t, uint32_t s) {
        detail::check_container_size_for_ext<sizeof(std::size_t)>(s);
        m_data.resize(static_cast<std::size_t>(s) + 1);
        m_data[0] = static_cast<char>(t);
    }
    ext(ext_ref const&);
    int8_t type() const {
        return static_cast<int8_t>(m_data[0]);
    }
    const char* data() const {
        return &m_data[1];
    }
    char* data() {
        return &m_data[1];
    }
    uint32_t size() const {
        return static_cast<uint32_t>(m_data.size()) - 1;
    }
    bool operator== (const ext& x) const {
        return m_data == x.m_data;
    }

    bool operator!= (const ext& x) const {
        return !(*this == x);
    }

    bool operator< (const ext& x) const {
        return m_data < x.m_data;
    }

    bool operator> (const ext& x) const {
        return m_data > x.m_data;
    }
private:
    std::vector<char> m_data;
    friend class ext_ref;
};

} // namespace type

namespace adaptor {

template <>
struct convert<clmdep_msgpack::type::ext> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, clmdep_msgpack::type::ext& v) const {
        if(o.type != clmdep_msgpack::type::EXT) {
            throw clmdep_msgpack::type_error();
        }
        v = clmdep_msgpack::type::ext(o.via.ext.type(), o.via.ext.data(), o.via.ext.size);
        return o;
    }
};

template <>
struct pack<clmdep_msgpack::type::ext> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::type::ext& v) const {
        // size limit has aleady been checked at ext's constructor
        uint32_t size = v.size();
        o.pack_ext(size, v.type());
        o.pack_ext_body(v.data(), size);
        return o;
    }
};

template <>
struct object_with_zone<clmdep_msgpack::type::ext> {
    void operator()(clmdep_msgpack::object::with_zone& o, const clmdep_msgpack::type::ext& v) const {
        // size limit has aleady been checked at ext's constructor
        uint32_t size = v.size();
        o.type = clmdep_msgpack::type::EXT;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size + 1));
        o.via.ext.ptr = ptr;
        o.via.ext.size = size;
        ptr[0] = static_cast<char>(v.type());
        std::memcpy(ptr + 1, v.data(), size);
    }
};

} // namespace adaptor

namespace type {

class ext_ref {
public:
    // ext_ref should be default constructible to support 'convert'.
    // A default constructed ext_ref object::m_ptr doesn't have the buffer to point to.
    // In order to avoid nullptr checking branches, m_ptr points to m_size.
    // So type() returns unspecified but valid value. It might be a zero because m_size
    // is initialized as zero, but shoudn't assume that.
    ext_ref() : m_ptr(static_cast<char*>(static_cast<void*>(&m_size))), m_size(0) {}
    ext_ref(const char* p, uint32_t s) :
        m_ptr(s == 0 ? static_cast<char*>(static_cast<void*>(&m_size)) : p),
        m_size(s == 0 ? 0 : s - 1) {
        detail::check_container_size_for_ext<sizeof(std::size_t)>(s);
    }

    // size limit has aleady been checked at ext's constructor
    ext_ref(ext const& x) : m_ptr(&x.m_data[0]), m_size(x.size()) {}

    const char* data() const {
        return m_ptr + 1;
    }

    uint32_t size() const {
        return m_size;
    }

    int8_t type() const {
        return static_cast<int8_t>(m_ptr[0]);
    }

    std::string str() const {
        return std::string(m_ptr + 1, m_size);
    }

    bool operator== (const ext_ref& x) const {
        return m_size == x.m_size && std::memcmp(m_ptr, x.m_ptr, m_size) == 0;
    }

    bool operator!= (const ext_ref& x) const {
        return !(*this == x);
    }

    bool operator< (const ext_ref& x) const {
        if (m_size < x.m_size) return true;
        if (m_size > x.m_size) return false;
        return std::memcmp(m_ptr, x.m_ptr, m_size) < 0;
    }

    bool operator> (const ext_ref& x) const {
        if (m_size > x.m_size) return true;
        if (m_size < x.m_size) return false;
        return std::memcmp(m_ptr, x.m_ptr, m_size) > 0;
    }
private:
    const char* m_ptr;
    uint32_t m_size;
    friend struct clmdep_msgpack::adaptor::object<clmdep_msgpack::type::ext_ref>;
};

inline ext::ext(ext_ref const& x) {
    // size limit has aleady been checked at ext_ref's constructor
    m_data.reserve(x.size() + 1);

    m_data.push_back(x.type());
    m_data.insert(m_data.end(), x.data(), x.data() + x.size());
}

} // namespace type

namespace adaptor {

template <>
struct convert<clmdep_msgpack::type::ext_ref> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, clmdep_msgpack::type::ext_ref& v) const {
        if(o.type != clmdep_msgpack::type::EXT) { throw clmdep_msgpack::type_error(); }
        v = clmdep_msgpack::type::ext_ref(o.via.ext.ptr, o.via.ext.size + 1);
        return o;
    }
};

template <>
struct pack<clmdep_msgpack::type::ext_ref> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::type::ext_ref& v) const {
        // size limit has aleady been checked at ext_ref's constructor
        uint32_t size = v.size();
        o.pack_ext(size, v.type());
        o.pack_ext_body(v.data(), size);
        return o;
    }
};

template <>
struct object<clmdep_msgpack::type::ext_ref> {
    void operator()(clmdep_msgpack::object& o, const clmdep_msgpack::type::ext_ref& v) const {
        // size limit has aleady been checked at ext_ref's constructor
        uint32_t size = v.size();
        o.type = clmdep_msgpack::type::EXT;
        o.via.ext.ptr = v.m_ptr;
        o.via.ext.size = size;
    }
};

template <>
struct object_with_zone<clmdep_msgpack::type::ext_ref> {
    void operator()(clmdep_msgpack::object::with_zone& o, const clmdep_msgpack::type::ext_ref& v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_EXT_HPP
