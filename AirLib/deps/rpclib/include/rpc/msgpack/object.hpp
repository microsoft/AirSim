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
#ifndef MSGPACK_OBJECT_HPP
#define MSGPACK_OBJECT_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/pack.hpp"
#include "rpc/msgpack/zone.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"

#include <cstring>
#include <stdexcept>
#include <typeinfo>
#include <limits>
#include <ostream>
#include <typeinfo>
#include <iomanip>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

class object_handle {
public:
    object_handle() {}

    object_handle(clmdep_msgpack::object const& obj, clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> z) :
        m_obj(obj), m_zone(clmdep_msgpack::move(z)) { }

#ifdef _MSC_VER
	object_handle(clmdep_msgpack::object const& obj, std::unique_ptr<clmdep_msgpack::zone>&& z) :
		m_obj(obj), m_zone(z.release()) { }
#endif

    // obsolete
    void set(clmdep_msgpack::object const& obj)
        { m_obj = obj; }

    const clmdep_msgpack::object& get() const
        { return m_obj; }

    template<typename T>
    T as() const { return m_obj.as<T>(); }

    template <typename T>
    void convert(T&& v) { m_obj.convert(std::forward<T>(v)); }

    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone>& zone()
        { return m_zone; }

    const clmdep_msgpack::unique_ptr<clmdep_msgpack::zone>& zone() const
        { return m_zone; }

#if defined(MSGPACK_USE_CPP03)
    struct object_handle_ref {
        object_handle_ref(object_handle* oh):m_oh(oh) {}
        object_handle* m_oh;
    };

    object_handle(object_handle& other):
        m_obj(other.m_obj),
        m_zone(clmdep_msgpack::move(other.m_zone)) {
    }

    object_handle(object_handle_ref ref):
        m_obj(ref.m_oh->m_obj),
        m_zone(clmdep_msgpack::move(ref.m_oh->m_zone)) {
    }

    object_handle& operator=(object_handle& other) {
        m_obj = other.m_obj;
        m_zone = clmdep_msgpack::move(other.m_zone);
        return *this;
    }

    object_handle& operator=(object_handle_ref ref) {
        m_obj = ref.m_oh->m_obj;
        m_zone = clmdep_msgpack::move(ref.m_oh->m_zone);
        return *this;
    }

    operator object_handle_ref() {
        return object_handle_ref(this);
    }
#endif // defined(MSGPACK_USE_CPP03)

    object_handle& assign(object_handle&& other) {
        m_obj = other.m_obj;
        m_zone = clmdep_msgpack::move(other.m_zone);
        return *this;
    }

private:
    clmdep_msgpack::object m_obj;
    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> m_zone;
};

namespace detail {

template <std::size_t N>
inline std::size_t add_ext_type_size(std::size_t size) {
    return size + 1;
}

template <>
inline std::size_t add_ext_type_size<4>(std::size_t size) {
    return size == 0xffffffff ? size : size + 1;
}

} // namespace detail

inline std::size_t aligned_zone_size(clmdep_msgpack::object const& obj) {
    std::size_t s = 0;
    switch (obj.type) {
    case clmdep_msgpack::type::ARRAY:
        s += sizeof(clmdep_msgpack::object) * obj.via.array.size;
        for (uint32_t i = 0; i < obj.via.array.size; ++i) {
            s += clmdep_msgpack::aligned_zone_size(obj.via.array.ptr[i]);
        }
        break;
    case clmdep_msgpack::type::MAP:
        s += sizeof(clmdep_msgpack::object_kv) * obj.via.map.size;
        for (uint32_t i = 0; i < obj.via.map.size; ++i) {
            s += clmdep_msgpack::aligned_zone_size(obj.via.map.ptr[i].key);
            s += clmdep_msgpack::aligned_zone_size(obj.via.map.ptr[i].val);
        }
        break;
    case clmdep_msgpack::type::EXT:
        s += clmdep_msgpack::aligned_size(
            detail::add_ext_type_size<sizeof(std::size_t)>(obj.via.ext.size));
        break;
    case clmdep_msgpack::type::STR:
        s += clmdep_msgpack::aligned_size(obj.via.str.size);
        break;
    case clmdep_msgpack::type::BIN:
        s += clmdep_msgpack::aligned_size(obj.via.bin.size);
        break;
    default:
        break;
    }
    return s;
}

inline object_handle clone(clmdep_msgpack::object const& obj) {
    std::size_t size = clmdep_msgpack::aligned_zone_size(obj);
    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> z(size == 0 ? nullptr : new clmdep_msgpack::zone(size));
    clmdep_msgpack::object newobj = z.get() ? clmdep_msgpack::object(obj, *z) : obj;
    return object_handle(newobj, clmdep_msgpack::move(z));
}

struct object::implicit_type {
    implicit_type(object const& o) : obj(o) { }
    ~implicit_type() { }

    template <typename T>
    operator T() { return obj.as<T>(); }

private:
    clmdep_msgpack::object const& obj;
};

namespace detail {
template <typename Stream, typename T>
struct packer_serializer {
    static clmdep_msgpack::packer<Stream>& pack(clmdep_msgpack::packer<Stream>& o, const T& v) {
        v.msgpack_pack(o);
        return o;
    }
};
} // namespace detail

// Adaptor functors' member functions definitions.
template <typename T, typename Enabler>
inline
clmdep_msgpack::object const&
clmdep_msgpack::adaptor::convert<T, Enabler>::operator()(clmdep_msgpack::object const& o, T& v) const {
    v.msgpack_unpack(o.convert());
    return o;
}

template <typename T, typename Enabler>
template <typename Stream>
inline
clmdep_msgpack::packer<Stream>&
clmdep_msgpack::adaptor::pack<T, Enabler>::operator()(clmdep_msgpack::packer<Stream>& o, T const& v) const {
    return clmdep_msgpack::detail::packer_serializer<Stream, T>::pack(o, v);
}

template <typename T, typename Enabler>
inline
void
clmdep_msgpack::adaptor::object_with_zone<T, Enabler>::operator()(clmdep_msgpack::object::with_zone& o, T const& v) const {
    v.msgpack_object(static_cast<clmdep_msgpack::object*>(&o), o.zone);
}

// Adaptor functor specialization to object
namespace adaptor {

template <>
struct convert<clmdep_msgpack::object> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, clmdep_msgpack::object& v) const {
        v = o;
        return o;
    }
};

template <>
struct pack<clmdep_msgpack::object> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, clmdep_msgpack::object const& v) const {
        switch(v.type) {
        case clmdep_msgpack::type::NIL:
            o.pack_nil();
            return o;

        case clmdep_msgpack::type::BOOLEAN:
            if(v.via.boolean) {
                o.pack_true();
            } else {
                o.pack_false();
            }
            return o;

        case clmdep_msgpack::type::POSITIVE_INTEGER:
            o.pack_uint64(v.via.u64);
            return o;

        case clmdep_msgpack::type::NEGATIVE_INTEGER:
            o.pack_int64(v.via.i64);
            return o;

        case clmdep_msgpack::type::FLOAT:
            o.pack_double(v.via.f64);
            return o;

        case clmdep_msgpack::type::STR:
            o.pack_str(v.via.str.size);
            o.pack_str_body(v.via.str.ptr, v.via.str.size);
            return o;

        case clmdep_msgpack::type::BIN:
            o.pack_bin(v.via.bin.size);
            o.pack_bin_body(v.via.bin.ptr, v.via.bin.size);
            return o;

        case clmdep_msgpack::type::EXT:
            o.pack_ext(v.via.ext.size, v.via.ext.type());
            o.pack_ext_body(v.via.ext.data(), v.via.ext.size);
            return o;

        case clmdep_msgpack::type::ARRAY:
            o.pack_array(v.via.array.size);
            for(clmdep_msgpack::object* p(v.via.array.ptr),
                    * const pend(v.via.array.ptr + v.via.array.size);
                p < pend; ++p) {
                clmdep_msgpack::operator<<(o, *p);
            }
            return o;

        case clmdep_msgpack::type::MAP:
            o.pack_map(v.via.map.size);
            for(clmdep_msgpack::object_kv* p(v.via.map.ptr),
                    * const pend(v.via.map.ptr + v.via.map.size);
                p < pend; ++p) {
                clmdep_msgpack::operator<<(o, p->key);
                clmdep_msgpack::operator<<(o, p->val);
            }
            return o;

        default:
            throw clmdep_msgpack::type_error();
        }
    }
};

template <>
struct object_with_zone<clmdep_msgpack::object> {
    void operator()(clmdep_msgpack::object::with_zone& o, clmdep_msgpack::object const& v) const {
        o.type = v.type;

        switch(v.type) {
        case clmdep_msgpack::type::NIL:
        case clmdep_msgpack::type::BOOLEAN:
        case clmdep_msgpack::type::POSITIVE_INTEGER:
        case clmdep_msgpack::type::NEGATIVE_INTEGER:
        case clmdep_msgpack::type::FLOAT:
            std::memcpy(&o.via, &v.via, sizeof(v.via));
            return;

        case clmdep_msgpack::type::STR: {
            char* ptr = static_cast<char*>(o.zone.allocate_align(v.via.str.size));
            o.via.str.ptr = ptr;
            o.via.str.size = v.via.str.size;
            std::memcpy(ptr, v.via.str.ptr, v.via.str.size);
            return;
        }

        case clmdep_msgpack::type::BIN: {
            char* ptr = static_cast<char*>(o.zone.allocate_align(v.via.bin.size));
            o.via.bin.ptr = ptr;
            o.via.bin.size = v.via.bin.size;
            std::memcpy(ptr, v.via.bin.ptr, v.via.bin.size);
            return;
        }

        case clmdep_msgpack::type::EXT: {
            char* ptr = static_cast<char*>(o.zone.allocate_align(v.via.ext.size + 1));
            o.via.ext.ptr = ptr;
            o.via.ext.size = v.via.ext.size;
            std::memcpy(ptr, v.via.ext.ptr, v.via.ext.size + 1);
            return;
        }

        case clmdep_msgpack::type::ARRAY:
            o.via.array.ptr = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object) * v.via.array.size));
            o.via.array.size = v.via.array.size;
            for (clmdep_msgpack::object
                     * po(o.via.array.ptr),
                     * pv(v.via.array.ptr),
                     * const pvend(v.via.array.ptr + v.via.array.size);
                 pv < pvend;
                 ++po, ++pv) {
                new (po) clmdep_msgpack::object(*pv, o.zone);
            }
            return;

        case clmdep_msgpack::type::MAP:
            o.via.map.ptr = (clmdep_msgpack::object_kv*)o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv) * v.via.map.size);
            o.via.map.size = v.via.map.size;
            for(clmdep_msgpack::object_kv
                    * po(o.via.map.ptr),
                    * pv(v.via.map.ptr),
                    * const pvend(v.via.map.ptr + v.via.map.size);
                pv < pvend;
                ++po, ++pv) {
                clmdep_msgpack::object_kv* kv = new (po) clmdep_msgpack::object_kv;
                new (&kv->key) clmdep_msgpack::object(pv->key, o.zone);
                new (&kv->val) clmdep_msgpack::object(pv->val, o.zone);
            }
            return;

        default:
            throw clmdep_msgpack::type_error();
        }

    }
};

// Adaptor functor specialization to object::with_zone

template <>
struct object_with_zone<clmdep_msgpack::object::with_zone> {
    void operator()(
        clmdep_msgpack::object::with_zone& o,
        clmdep_msgpack::object::with_zone const& v) const {
        o << static_cast<clmdep_msgpack::object const&>(v);
    }
};


} // namespace adaptor


// obsolete
template <typename Type>
class define : public Type {
public:
    typedef Type msgpack_type;
    typedef define<Type> define_type;

    define() {}
    define(const msgpack_type& v) : msgpack_type(v) {}

    template <typename Packer>
    void msgpack_pack(Packer& o) const
    {
        clmdep_msgpack::operator<<(o, static_cast<const msgpack_type&>(*this));
    }

    void msgpack_unpack(object const& o)
    {
        clmdep_msgpack::operator>>(o, static_cast<msgpack_type&>(*this));
    }
};

// deconvert operator

template <typename Stream>
template <typename T>
inline clmdep_msgpack::packer<Stream>& packer<Stream>::pack(const T& v)
{
    clmdep_msgpack::operator<<(*this, v);
    return *this;
}

inline bool operator==(const clmdep_msgpack::object& x, const clmdep_msgpack::object& y)
{
    if(x.type != y.type) { return false; }

    switch(x.type) {
    case clmdep_msgpack::type::NIL:
        return true;

    case clmdep_msgpack::type::BOOLEAN:
        return x.via.boolean == y.via.boolean;

    case clmdep_msgpack::type::POSITIVE_INTEGER:
        return x.via.u64 == y.via.u64;

    case clmdep_msgpack::type::NEGATIVE_INTEGER:
        return x.via.i64 == y.via.i64;

    case clmdep_msgpack::type::FLOAT:
        return x.via.f64 == y.via.f64;

    case clmdep_msgpack::type::STR:
        return x.via.str.size == y.via.str.size &&
            std::memcmp(x.via.str.ptr, y.via.str.ptr, x.via.str.size) == 0;

    case clmdep_msgpack::type::BIN:
        return x.via.bin.size == y.via.bin.size &&
            std::memcmp(x.via.bin.ptr, y.via.bin.ptr, x.via.bin.size) == 0;

    case clmdep_msgpack::type::EXT:
        return x.via.ext.size == y.via.ext.size &&
            std::memcmp(x.via.ext.ptr, y.via.ext.ptr, x.via.ext.size) == 0;

    case clmdep_msgpack::type::ARRAY:
        if(x.via.array.size != y.via.array.size) {
            return false;
        } else if(x.via.array.size == 0) {
            return true;
        } else {
            clmdep_msgpack::object* px = x.via.array.ptr;
            clmdep_msgpack::object* const pxend = x.via.array.ptr + x.via.array.size;
            clmdep_msgpack::object* py = y.via.array.ptr;
            do {
                if(!(*px == *py)) {
                    return false;
                }
                ++px;
                ++py;
            } while(px < pxend);
            return true;
        }

    case clmdep_msgpack::type::MAP:
        if(x.via.map.size != y.via.map.size) {
            return false;
        } else if(x.via.map.size == 0) {
            return true;
        } else {
            clmdep_msgpack::object_kv* px = x.via.map.ptr;
            clmdep_msgpack::object_kv* const pxend = x.via.map.ptr + x.via.map.size;
            clmdep_msgpack::object_kv* py = y.via.map.ptr;
            do {
                if(!(px->key == py->key) || !(px->val == py->val)) {
                    return false;
                }
                ++px;
                ++py;
            } while(px < pxend);
            return true;
        }

    default:
        return false;
    }
}

template <typename T>
inline bool operator==(const clmdep_msgpack::object& x, const T& y)
try {
    return x == clmdep_msgpack::object(y);
} catch (clmdep_msgpack::type_error&) {
    return false;
}

inline bool operator!=(const clmdep_msgpack::object& x, const clmdep_msgpack::object& y)
{ return !(x == y); }

template <typename T>
inline bool operator==(const T& y, const clmdep_msgpack::object& x)
{ return x == y; }

template <typename T>
inline bool operator!=(const clmdep_msgpack::object& x, const T& y)
{ return !(x == y); }

template <typename T>
inline bool operator!=(const T& y, const clmdep_msgpack::object& x)
{ return x != y; }


inline clmdep_msgpack::object::implicit_type object::convert() const
{
    return clmdep_msgpack::object::implicit_type(*this);
}

template <typename T>
inline T& object::convert(T& v) const
{
    clmdep_msgpack::operator>>(*this, v);
    return v;
}

template <typename T>
inline T* object::convert(T* v) const
{
    convert(*v);
    return v;
}

template <typename T>
inline bool object::convert_if_not_nil(T& v) const
{
    if (is_nil()) {
        return false;
    }
    convert(v);
    return true;
}

#if defined(MSGPACK_USE_CPP03)

template <typename T>
inline T object::as() const
{
    T v;
    convert(v);
    return v;
}

#else  // defined(MSGPACK_USE_CPP03)

template <typename T>
inline typename std::enable_if<clmdep_msgpack::has_as<T>::value, T>::type object::as() const {
    return clmdep_msgpack::adaptor::as<T>()(*this);
}

template <typename T>
inline typename std::enable_if<!clmdep_msgpack::has_as<T>::value, T>::type object::as() const {
    T v;
    convert(v);
    return v;
}

#endif // defined(MSGPACK_USE_CPP03)

inline object::object()
{
    type = clmdep_msgpack::type::NIL;
}

template <typename T>
inline object::object(const T& v)
{
    clmdep_msgpack::operator<<(*this, v);
}

template <typename T>
inline object& object::operator=(const T& v)
{
    *this = object(v);
    return *this;
}

template <typename T>
object::object(const T& v, clmdep_msgpack::zone& z)
{
    with_zone oz(z);
    clmdep_msgpack::operator<<(oz, v);
    type = oz.type;
    via = oz.via;
}

template <typename T>
object::object(const T& v, clmdep_msgpack::zone* z)
{
    with_zone oz(*z);
    clmdep_msgpack::operator<<(oz, v);
    type = oz.type;
    via = oz.via;
}


inline object::object(const msgpack_object& o)
{
    // FIXME beter way?
    std::memcpy(this, &o, sizeof(o));
}

inline void operator<< (clmdep_msgpack::object& o, const msgpack_object& v)
{
    // FIXME beter way?
    std::memcpy(&o, &v, sizeof(v));
}

inline object::operator msgpack_object() const
{
    // FIXME beter way?
    msgpack_object obj;
    std::memcpy(&obj, this, sizeof(obj));
    return obj;
}


// obsolete
template <typename T>
inline void convert(T& v, clmdep_msgpack::object const& o)
{
    o.convert(v);
}

// obsolete
template <typename Stream, typename T>
inline void pack(clmdep_msgpack::packer<Stream>& o, const T& v)
{
    o.pack(v);
}

// obsolete
template <typename Stream, typename T>
inline void pack_copy(clmdep_msgpack::packer<Stream>& o, T v)
{
    pack(o, v);
}


template <typename Stream>
inline clmdep_msgpack::packer<Stream>& operator<< (clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::object& v)
{
    switch(v.type) {
    case clmdep_msgpack::type::NIL:
        o.pack_nil();
        return o;

    case clmdep_msgpack::type::BOOLEAN:
        if(v.via.boolean) {
            o.pack_true();
        } else {
            o.pack_false();
        }
        return o;

    case clmdep_msgpack::type::POSITIVE_INTEGER:
        o.pack_uint64(v.via.u64);
        return o;

    case clmdep_msgpack::type::NEGATIVE_INTEGER:
        o.pack_int64(v.via.i64);
        return o;

    case clmdep_msgpack::type::FLOAT:
        o.pack_double(v.via.f64);
        return o;

    case clmdep_msgpack::type::STR:
        o.pack_str(v.via.str.size);
        o.pack_str_body(v.via.str.ptr, v.via.str.size);
        return o;

    case clmdep_msgpack::type::BIN:
        o.pack_bin(v.via.bin.size);
        o.pack_bin_body(v.via.bin.ptr, v.via.bin.size);
        return o;

    case clmdep_msgpack::type::EXT:
        o.pack_ext(v.via.ext.size, v.via.ext.type());
        o.pack_ext_body(v.via.ext.data(), v.via.ext.size);
        return o;

    case clmdep_msgpack::type::ARRAY:
        o.pack_array(v.via.array.size);
        for(clmdep_msgpack::object* p(v.via.array.ptr),
                * const pend(v.via.array.ptr + v.via.array.size);
                p < pend; ++p) {
            clmdep_msgpack::operator<<(o, *p);
        }
        return o;

    case clmdep_msgpack::type::MAP:
        o.pack_map(v.via.map.size);
        for(clmdep_msgpack::object_kv* p(v.via.map.ptr),
                * const pend(v.via.map.ptr + v.via.map.size);
                p < pend; ++p) {
            clmdep_msgpack::operator<<(o, p->key);
            clmdep_msgpack::operator<<(o, p->val);
        }
        return o;

    default:
        throw clmdep_msgpack::type_error();
    }
}

template <typename Stream>
clmdep_msgpack::packer<Stream>& operator<< (clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::object::with_zone& v)
{
    return o << static_cast<clmdep_msgpack::object>(v);
}

inline std::ostream& operator<< (std::ostream& s, const clmdep_msgpack::object& o)
{
    switch(o.type) {
    case clmdep_msgpack::type::NIL:
        s << "nil";
        break;

    case clmdep_msgpack::type::BOOLEAN:
        s << (o.via.boolean ? "true" : "false");
        break;

    case clmdep_msgpack::type::POSITIVE_INTEGER:
        s << o.via.u64;
        break;

    case clmdep_msgpack::type::NEGATIVE_INTEGER:
        s << o.via.i64;
        break;

    case clmdep_msgpack::type::FLOAT:
        s << o.via.f64;
        break;

    case clmdep_msgpack::type::STR:
        s << '"';
        for (uint32_t i = 0; i < o.via.str.size; ++i) {
            char c = o.via.str.ptr[i];
            switch (c) {
            case '\\':
                s << "\\\\";
                break;
            case '"':
                s << "\\\"";
                break;
            case '/':
                s << "\\/";
                break;
            case '\b':
                s << "\\b";
                break;
            case '\f':
                s << "\\f";
                break;
            case '\n':
                s << "\\n";
                break;
            case '\r':
                s << "\\r";
                break;
            case '\t':
                s << "\\t";
                break;
            default: {
                unsigned int code = static_cast<unsigned int>(c);
                if (code < 0x20 || code == 0x7f) {
                    s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (code & 0xff);
                }
                else {
                    s << c;
                }
            } break;
            }
        }
        s << '"';
        break;

    case clmdep_msgpack::type::BIN:
        (s << '"').write(o.via.bin.ptr, o.via.bin.size) << '"';
        break;

    case clmdep_msgpack::type::EXT:
        s << "EXT";
        break;

    case clmdep_msgpack::type::ARRAY:
        s << "[";
        if(o.via.array.size != 0) {
            clmdep_msgpack::object* p(o.via.array.ptr);
            s << *p;
            ++p;
            for(clmdep_msgpack::object* const pend(o.via.array.ptr + o.via.array.size);
                    p < pend; ++p) {
                s << ", " << *p;
            }
        }
        s << "]";
        break;

    case clmdep_msgpack::type::MAP:
        s << "{";
        if(o.via.map.size != 0) {
            clmdep_msgpack::object_kv* p(o.via.map.ptr);
            s << p->key << ':' << p->val;
            ++p;
            for(clmdep_msgpack::object_kv* const pend(o.via.map.ptr + o.via.map.size);
                    p < pend; ++p) {
                s << ", " << p->key << ':' << p->val;
            }
        }
        s << "}";
        break;

    default:
        // FIXME
        s << "#<UNKNOWN " << static_cast<uint16_t>(o.type) << ">";
    }
    return s;
}

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#include "rpc/msgpack/type.hpp"

#endif /* msgpack/object.hpp */
