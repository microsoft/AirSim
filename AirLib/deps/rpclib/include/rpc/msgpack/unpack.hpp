//
// MessagePack for C++ deserializing routine
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
#ifndef MSGPACK_UNPACK_HPP
#define MSGPACK_UNPACK_HPP

#include "rpc/msgpack/versioning.hpp"
#include "object.hpp"
#include "zone.hpp"
#include "unpack_define.h"
#include "cpp_config.hpp"
#include "sysdep.h"

#include <memory>
#include <stdexcept>

#if !defined(MSGPACK_USE_CPP03)
#include <atomic>
#endif


#if defined(_MSC_VER)
// avoiding confliction std::max, std::min, and macro in windows.h
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif // defined(_MSC_VER)

#ifdef _msgpack_atomic_counter_header
#include _msgpack_atomic_counter_header
#endif


const size_t COUNTER_SIZE = sizeof(_msgpack_atomic_counter_t);

#ifndef MSGPACK_UNPACKER_INIT_BUFFER_SIZE
#define MSGPACK_UNPACKER_INIT_BUFFER_SIZE (64*1024)
#endif

#ifndef MSGPACK_UNPACKER_RESERVE_SIZE
#define MSGPACK_UNPACKER_RESERVE_SIZE (32*1024)
#endif


// backward compatibility
#ifndef MSGPACK_UNPACKER_DEFAULT_INITIAL_BUFFER_SIZE
#define MSGPACK_UNPACKER_DEFAULT_INITIAL_BUFFER_SIZE MSGPACK_UNPACKER_INIT_BUFFER_SIZE
#endif


namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

typedef bool (*unpack_reference_func)(clmdep_msgpack::type::object_type, std::size_t, void*);

struct unpack_error : public std::runtime_error {
    explicit unpack_error(const std::string& msg)
        :std::runtime_error(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    explicit unpack_error(const char* msg):
        std::runtime_error(msg) {}
#endif // !defined(MSGPACK_USE_CPP03)
};

struct parse_error : public unpack_error {
    explicit parse_error(const std::string& msg)
        :unpack_error(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    explicit parse_error(const char* msg)
        :unpack_error(msg) {}
#endif // !defined(MSGPACK_USE_CPP03)
};

struct insufficient_bytes : public unpack_error {
    explicit insufficient_bytes(const std::string& msg)
        :unpack_error(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    explicit insufficient_bytes(const char* msg)
        :unpack_error(msg) {}
#endif // !defined(MSGPACK_USE_CPP03)
};

struct size_overflow : public unpack_error {
    explicit size_overflow(const std::string& msg)
        :unpack_error(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    explicit size_overflow(const char* msg)
        :unpack_error(msg) {}
#endif
};

struct array_size_overflow : public size_overflow {
    array_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    array_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

struct map_size_overflow : public size_overflow {
    map_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    map_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

struct str_size_overflow : public size_overflow {
    str_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    str_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

struct bin_size_overflow : public size_overflow {
    bin_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    bin_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

struct ext_size_overflow : public size_overflow {
    ext_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    ext_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

struct depth_size_overflow : public size_overflow {
    depth_size_overflow(const std::string& msg)
        :size_overflow(msg) {}
#if !defined(MSGPACK_USE_CPP03)
    depth_size_overflow(const char* msg)
        :size_overflow(msg) {}
#endif
};

class unpack_limit {
public:
    unpack_limit(
        std::size_t array = 0xffffffff,
        std::size_t map = 0xffffffff,
        std::size_t str = 0xffffffff,
        std::size_t bin = 0xffffffff,
        std::size_t ext = 0xffffffff,
        std::size_t depth = 0xffffffff)
        :array_(array),
         map_(map),
         str_(str),
         bin_(bin),
         ext_(ext),
         depth_(depth) {}
    std::size_t array() const { return array_; }
    std::size_t map() const { return map_; }
    std::size_t str() const { return str_; }
    std::size_t bin() const { return bin_; }
    std::size_t ext() const { return ext_; }
    std::size_t depth() const { return depth_; }

private:
    std::size_t array_;
    std::size_t map_;
    std::size_t str_;
    std::size_t bin_;
    std::size_t ext_;
    std::size_t depth_;
};

namespace detail {

class unpack_user {
public:
    unpack_user(unpack_reference_func f = nullptr,
                void* user_data = nullptr,
                unpack_limit const& limit = unpack_limit())
        :m_func(f), m_user_data(user_data), m_limit(limit) {}
    clmdep_msgpack::zone const& zone() const { return *m_zone; }
    clmdep_msgpack::zone& zone() { return *m_zone; }
    void set_zone(clmdep_msgpack::zone& zone) { m_zone = &zone; }
    bool referenced() const { return m_referenced; }
    void set_referenced(bool referenced) { m_referenced = referenced; }
    unpack_reference_func reference_func() const { return m_func; }
    void* user_data() const { return m_user_data; }
    unpack_limit const& limit() const { return m_limit; }
    unpack_limit& limit() { return m_limit; }

private:
    clmdep_msgpack::zone* m_zone;
    bool m_referenced;
    unpack_reference_func m_func;
    void* m_user_data;
    unpack_limit m_limit;
};

inline void unpack_uint8(uint8_t d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }

inline void unpack_uint16(uint16_t d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }

inline void unpack_uint32(uint32_t d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }

inline void unpack_uint64(uint64_t d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }

inline void unpack_int8(int8_t d, clmdep_msgpack::object& o)
{ if(d >= 0) { o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }
        else { o.type = clmdep_msgpack::type::NEGATIVE_INTEGER; o.via.i64 = d; } }

inline void unpack_int16(int16_t d, clmdep_msgpack::object& o)
{ if(d >= 0) { o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }
        else { o.type = clmdep_msgpack::type::NEGATIVE_INTEGER; o.via.i64 = d; } }

inline void unpack_int32(int32_t d, clmdep_msgpack::object& o)
{ if(d >= 0) { o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }
        else { o.type = clmdep_msgpack::type::NEGATIVE_INTEGER; o.via.i64 = d; } }

inline void unpack_int64(int64_t d, clmdep_msgpack::object& o)
{ if(d >= 0) { o.type = clmdep_msgpack::type::POSITIVE_INTEGER; o.via.u64 = d; }
        else { o.type = clmdep_msgpack::type::NEGATIVE_INTEGER; o.via.i64 = d; } }

inline void unpack_float(float d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::FLOAT; o.via.f64 = d; }

inline void unpack_double(double d, clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::FLOAT; o.via.f64 = d; }

inline void unpack_nil(clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::NIL; }

inline void unpack_true(clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::BOOLEAN; o.via.boolean = true; }

inline void unpack_false(clmdep_msgpack::object& o)
{ o.type = clmdep_msgpack::type::BOOLEAN; o.via.boolean = false; }

struct unpack_array {
    void operator()(unpack_user& u, uint32_t n, clmdep_msgpack::object& o) const {
        if (n > u.limit().array()) throw clmdep_msgpack::array_size_overflow("array size overflow");
        o.type = clmdep_msgpack::type::ARRAY;
        o.via.array.size = 0;
        o.via.array.ptr = static_cast<clmdep_msgpack::object*>(u.zone().allocate_align(n*sizeof(clmdep_msgpack::object)));
    }
};

inline void unpack_array_item(clmdep_msgpack::object& c, clmdep_msgpack::object const& o)
{
#if defined(__GNUC__) && !defined(__clang__)
    std::memcpy(&c.via.array.ptr[c.via.array.size++], &o, sizeof(clmdep_msgpack::object));
#else  /* __GNUC__ && !__clang__ */
    c.via.array.ptr[c.via.array.size++] = o;
#endif /* __GNUC__ && !__clang__ */
}

struct unpack_map {
    void operator()(unpack_user& u, uint32_t n, clmdep_msgpack::object& o) const {
        if (n > u.limit().map()) throw clmdep_msgpack::map_size_overflow("map size overflow");
        o.type = clmdep_msgpack::type::MAP;
        o.via.map.size = 0;
        o.via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(u.zone().allocate_align(n*sizeof(clmdep_msgpack::object_kv)));
    }
};

inline void unpack_map_item(clmdep_msgpack::object& c, clmdep_msgpack::object const& k, clmdep_msgpack::object const& v)
{
#if defined(__GNUC__) && !defined(__clang__)
    std::memcpy(&c.via.map.ptr[c.via.map.size].key, &k, sizeof(clmdep_msgpack::object));
    std::memcpy(&c.via.map.ptr[c.via.map.size].val, &v, sizeof(clmdep_msgpack::object));
#else  /* __GNUC__ && !__clang__ */
    c.via.map.ptr[c.via.map.size].key = k;
    c.via.map.ptr[c.via.map.size].val = v;
#endif /* __GNUC__ && !__clang__ */
    ++c.via.map.size;
}

inline void unpack_str(unpack_user& u, const char* p, uint32_t l, clmdep_msgpack::object& o)
{
    o.type = clmdep_msgpack::type::STR;
    if (u.reference_func() && u.reference_func()(o.type, l, u.user_data())) {
        o.via.str.ptr = p;
        u.set_referenced(true);
    }
    else {
        if (l > u.limit().str()) throw clmdep_msgpack::str_size_overflow("str size overflow");
        char* tmp = static_cast<char*>(u.zone().allocate_align(l));
        std::memcpy(tmp, p, l);
        o.via.str.ptr = tmp;
    }
    o.via.str.size = l;
}

inline void unpack_bin(unpack_user& u, const char* p, uint32_t l, clmdep_msgpack::object& o)
{
    o.type = clmdep_msgpack::type::BIN;
    if (u.reference_func() && u.reference_func()(o.type, l, u.user_data())) {
        o.via.bin.ptr = p;
        u.set_referenced(true);
    }
    else {
        if (l > u.limit().bin()) throw clmdep_msgpack::bin_size_overflow("bin size overflow");
        char* tmp = static_cast<char*>(u.zone().allocate_align(l));
        std::memcpy(tmp, p, l);
        o.via.bin.ptr = tmp;
    }
    o.via.bin.size = l;
}

inline void unpack_ext(unpack_user& u, const char* p, std::size_t l, clmdep_msgpack::object& o)
{
    o.type = clmdep_msgpack::type::EXT;
    if (u.reference_func() && u.reference_func()(o.type, l, u.user_data())) {
        o.via.ext.ptr = p;
        u.set_referenced(true);
    }
    else {
        if (l > u.limit().ext()) throw clmdep_msgpack::ext_size_overflow("ext size overflow");
        char* tmp = static_cast<char*>(u.zone().allocate_align(l));
        std::memcpy(tmp, p, l);
        o.via.ext.ptr = tmp;
    }
    o.via.ext.size = static_cast<uint32_t>(l - 1);
}


class unpack_stack {
public:
    clmdep_msgpack::object const& obj() const { return m_obj; }
    clmdep_msgpack::object& obj() { return m_obj; }
    void set_obj(clmdep_msgpack::object const& obj) { m_obj = obj; }
    std::size_t count() const { return m_count; }
    void set_count(std::size_t count) { m_count = count; }
    std::size_t decr_count() { return --m_count; }
    uint32_t container_type() const { return m_container_type; }
    void set_container_type(uint32_t container_type) { m_container_type = container_type; }
    clmdep_msgpack::object const& map_key() const { return m_map_key; }
    void set_map_key(clmdep_msgpack::object const& map_key) { m_map_key = map_key; }
private:
    clmdep_msgpack::object m_obj;
    std::size_t m_count;
    uint32_t m_container_type;
    clmdep_msgpack::object m_map_key;
};

inline void init_count(void* buffer)
{
#if defined(MSGPACK_USE_CPP03)
    *reinterpret_cast<volatile _msgpack_atomic_counter_t*>(buffer) = 1;
#else  // defined(MSGPACK_USE_CPP03)
    new (buffer) std::atomic<unsigned int>(1);
#endif // defined(MSGPACK_USE_CPP03)
}

inline void decr_count(void* buffer)
{
#if defined(MSGPACK_USE_CPP03)
    if(_msgpack_sync_decr_and_fetch(reinterpret_cast<volatile _msgpack_atomic_counter_t*>(buffer)) == 0) {
        free(buffer);
    }
#else  // defined(MSGPACK_USE_CPP03)
    if (--*reinterpret_cast<std::atomic<unsigned int>*>(buffer) == 0) {
        free(buffer);
    }
#endif // defined(MSGPACK_USE_CPP03)
}

inline void incr_count(void* buffer)
{
#if defined(MSGPACK_USE_CPP03)
    _msgpack_sync_incr_and_fetch(reinterpret_cast<volatile _msgpack_atomic_counter_t*>(buffer));
#else  // defined(MSGPACK_USE_CPP03)
    ++*reinterpret_cast<std::atomic<unsigned int>*>(buffer);
#endif // defined(MSGPACK_USE_CPP03)
}

#if defined(MSGPACK_USE_CPP03)
inline _msgpack_atomic_counter_t get_count(void* buffer)
{
    return *reinterpret_cast<volatile _msgpack_atomic_counter_t*>(buffer);
}
#else  // defined(MSGPACK_USE_CPP03)
inline std::atomic<unsigned int> const& get_count(void* buffer)
{
    return *reinterpret_cast<std::atomic<unsigned int>*>(buffer);
}
#endif // defined(MSGPACK_USE_CPP03)

struct fix_tag {
    char f1[65]; // FIXME unique size is required. or use is_same meta function.
};

template <typename T>
struct value {
    typedef T type;
};
template <>
struct value<fix_tag> {
    typedef uint32_t type;
};

template <typename T>
inline void load(uint32_t& dst, const char* n, typename clmdep_msgpack::enable_if<sizeof(T) == sizeof(fix_tag)>::type* = nullptr) {
    dst = static_cast<uint32_t>(*reinterpret_cast<const uint8_t*>(n)) & 0x0f;
}

template <typename T>
inline void load(T& dst, const char* n, typename clmdep_msgpack::enable_if<sizeof(T) == 1>::type* = nullptr) {
    dst = static_cast<T>(*reinterpret_cast<const uint8_t*>(n));
}

template <typename T>
inline void load(T& dst, const char* n, typename clmdep_msgpack::enable_if<sizeof(T) == 2>::type* = nullptr) {
    _msgpack_load16(T, n, &dst);
}

template <typename T>
inline void load(T& dst, const char* n, typename clmdep_msgpack::enable_if<sizeof(T) == 4>::type* = nullptr) {
    _msgpack_load32(T, n, &dst);
}

template <typename T>
inline void load(T& dst, const char* n, typename clmdep_msgpack::enable_if<sizeof(T) == 8>::type* = nullptr) {
    _msgpack_load64(T, n, &dst);
}

class context {
public:
    context(unpack_reference_func f, void* user_data, unpack_limit const& limit)
        :m_trail(0), m_user(f, user_data, limit), m_cs(MSGPACK_CS_HEADER)
    {
        m_stack.reserve(MSGPACK_EMBED_STACK_SIZE);
        m_stack.push_back(unpack_stack());
    }

    void init()
    {
        m_cs = MSGPACK_CS_HEADER;
        m_trail = 0;
        m_stack.resize(1);
        m_stack[0].set_obj(clmdep_msgpack::object());
    }

    clmdep_msgpack::object const& data() const
    {
        return m_stack[0].obj();
    }

    unpack_user& user()
    {
        return m_user;
    }

    unpack_user const& user() const
    {
        return m_user;
    }

    int execute(const char* data, std::size_t len, std::size_t& off);

private:
    template <typename T>
    static uint32_t next_cs(T p)
    {
        return static_cast<uint32_t>(*p) & 0x1f;
    }

    template <typename T, typename Func>
    int push_aggregate(
        Func const& f,
        uint32_t container_type,
        clmdep_msgpack::object& obj,
        const char* load_pos,
        std::size_t& off) {
        typename value<T>::type tmp;
        load<T>(tmp, load_pos);
        f(m_user, tmp, m_stack.back().obj());
        if(tmp == 0) {
            obj = m_stack.back().obj();
            int ret = push_proc(obj, off);
            if (ret != 0) return ret;
        }
        else {
            m_stack.back().set_container_type(container_type);
            m_stack.back().set_count(tmp);
            if (m_stack.size() <= m_user.limit().depth()) {
                m_stack.push_back(unpack_stack());
            }
            else {
                throw clmdep_msgpack::depth_size_overflow("depth size overflow");
            }
            m_cs = MSGPACK_CS_HEADER;
            ++m_current;
        }
        return 0;
    }

    int push_item(clmdep_msgpack::object& obj) {
        bool finish = false;
        while (!finish) {
            if(m_stack.size() == 1) {
                return 1;
            }
            unpack_stack& sp = *(m_stack.end() - 2);
            switch(sp.container_type()) {
            case MSGPACK_CT_ARRAY_ITEM:
                unpack_array_item(sp.obj(), obj);
                if(sp.decr_count() == 0) {
                    obj = sp.obj();
                    m_stack.pop_back();
                }
                else {
                    finish = true;
                }
                break;
            case MSGPACK_CT_MAP_KEY:
                sp.set_map_key(obj);
                sp.set_container_type(MSGPACK_CT_MAP_VALUE);
                finish = true;
                break;
            case MSGPACK_CT_MAP_VALUE:
                unpack_map_item(sp.obj(), sp.map_key(), obj);
                if(sp.decr_count() == 0) {
                    obj = sp.obj();
                    m_stack.pop_back();
                }
                else {
                    sp.set_container_type(MSGPACK_CT_MAP_KEY);
                    finish = true;
                }
                break;
            default:
                return -1;
            }
        }
        return 0;
    }

    int push_proc(clmdep_msgpack::object& obj, std::size_t& off) {
        int ret = push_item(obj);
        if (ret > 0) {
            m_stack[0].set_obj(obj);
            ++m_current;
            /*printf("-- finish --\n"); */
            off = m_current - m_start;
        }
        else if (ret < 0) {
            off = m_current - m_start;
        }
        else {
            m_cs = MSGPACK_CS_HEADER;
            ++m_current;
        }
        return ret;
    }

    template <std::size_t N>
    static void check_ext_size(std::size_t /*size*/) {
    }

private:
    char const* m_start;
    char const* m_current;

    std::size_t m_trail;
    unpack_user m_user;
    uint32_t m_cs;
    std::vector<unpack_stack> m_stack;
};

template <>
inline void context::check_ext_size<4>(std::size_t size) {
    if (size == 0xffffffff) throw clmdep_msgpack::ext_size_overflow("ext size overflow");
}

inline int context::execute(const char* data, std::size_t len, std::size_t& off)
{
    assert(len >= off);

    m_start = data;
    m_current = data + off;
    const char* const pe = data + len;
    const char* n = nullptr;

    clmdep_msgpack::object obj;

    if(m_current == pe) {
        off = m_current - m_start;
        return 0;
    }
    bool fixed_trail_again = false;
    do {
        if (m_cs == MSGPACK_CS_HEADER) {
            fixed_trail_again = false;
            int selector = *reinterpret_cast<const unsigned char*>(m_current);
            if (0x00 <= selector && selector <= 0x7f) { // Positive Fixnum
                unpack_uint8(*reinterpret_cast<const uint8_t*>(m_current), obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } else if(0xe0 <= selector && selector <= 0xff) { // Negative Fixnum
                unpack_int8(*reinterpret_cast<const int8_t*>(m_current), obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } else if (0xc4 <= selector && selector <= 0xdf) {
                const uint32_t trail[] = {
                    1, // bin     8  0xc4
                    2, // bin    16  0xc5
                    4, // bin    32  0xc6
                    1, // ext     8  0xc7
                    2, // ext    16  0xc8
                    4, // ext    32  0xc9
                    4, // float  32  0xca
                    8, // float  64  0xcb
                    1, // uint    8  0xcc
                    2, // uint   16  0xcd
                    4, // uint   32  0xce
                    8, // uint   64  0xcf
                    1, // int     8  0xd0
                    2, // int    16  0xd1
                    4, // int    32  0xd2
                    8, // int    64  0xd3
                    2, // fixext  1  0xd4
                    3, // fixext  2  0xd5
                    5, // fixext  4  0xd6
                    9, // fixext  8  0xd7
                    17,// fixext 16  0xd8
                    1, // str     8  0xd9
                    2, // str    16  0xda
                    4, // str    32  0xdb
                    2, // array  16  0xdc
                    4, // array  32  0xdd
                    2, // map    16  0xde
                    4, // map    32  0xdf
                };
                m_trail = trail[selector - 0xc4];
                m_cs = next_cs(m_current);
                fixed_trail_again = true;
            } else if(0xa0 <= selector && selector <= 0xbf) { // FixStr
                m_trail = static_cast<uint32_t>(*m_current) & 0x1f;
                if(m_trail == 0) {
                    unpack_str(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_STR_VALUE;
                    fixed_trail_again = true;
                }

            } else if(0x90 <= selector && selector <= 0x9f) { // FixArray
                int ret = push_aggregate<fix_tag>(
                    unpack_array(), MSGPACK_CT_ARRAY_ITEM, obj, m_current, off);
                if (ret != 0) return ret;
            } else if(0x80 <= selector && selector <= 0x8f) { // FixMap
                int ret = push_aggregate<fix_tag>(
                    unpack_map(), MSGPACK_CT_MAP_KEY, obj, m_current, off);
                if (ret != 0) return ret;
            } else if(selector == 0xc2) { // false
                unpack_false(obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } else if(selector == 0xc3) { // true
                unpack_true(obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } else if(selector == 0xc0) { // nil
                unpack_nil(obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } else {
                off = m_current - m_start;
                return -1;
            }
            // end MSGPACK_CS_HEADER
        }
        if (m_cs != MSGPACK_CS_HEADER || fixed_trail_again) {
            if (fixed_trail_again) {
                ++m_current;
                fixed_trail_again = false;
            }
            if(static_cast<std::size_t>(pe - m_current) < m_trail) {
                off = m_current - m_start;
                return 0;
            }
            n = m_current;
            m_current += m_trail - 1;
            switch(m_cs) {
                //case MSGPACK_CS_
                //case MSGPACK_CS_
            case MSGPACK_CS_FLOAT: {
                union { uint32_t i; float f; } mem;
                load<uint32_t>(mem.i, n);
                unpack_float(mem.f, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_DOUBLE: {
                union { uint64_t i; double f; } mem;
                load<uint64_t>(mem.i, n);
#if defined(TARGET_OS_IPHONE)
                // ok
#elif defined(__arm__) && !(__ARM_EABI__) // arm-oabi
                // https://github.com/msgpack/msgpack-perl/pull/1
                mem.i = (mem.i & 0xFFFFFFFFUL) << 32UL | (mem.i >> 32UL);
#endif
                unpack_double(mem.f, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_UINT_8: {
                uint8_t tmp;
                load<uint8_t>(tmp, n);
                unpack_uint8(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_UINT_16: {
                uint16_t tmp;
                load<uint16_t>(tmp, n);
                unpack_uint16(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_UINT_32: {
                uint32_t tmp;
                load<uint32_t>(tmp, n);
                unpack_uint32(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_UINT_64: {
                uint64_t tmp;
                load<uint64_t>(tmp, n);
                unpack_uint64(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_INT_8: {
                int8_t tmp;
                load<int8_t>(tmp, n);
                unpack_int8(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_INT_16: {
                int16_t tmp;
                load<int16_t>(tmp, n);
                unpack_int16(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_INT_32: {
                int32_t tmp;
                load<int32_t>(tmp, n);
                unpack_int32(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_INT_64: {
                int64_t tmp;
                load<int64_t>(tmp, n);
                unpack_int64(tmp, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_FIXEXT_1: {
                unpack_ext(m_user, n, 1+1, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_FIXEXT_2: {
                unpack_ext(m_user, n, 2+1, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_FIXEXT_4: {
                unpack_ext(m_user, n, 4+1, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_FIXEXT_8: {
                unpack_ext(m_user, n, 8+1, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_FIXEXT_16: {
                unpack_ext(m_user, n, 16+1, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_STR_8: {
                uint8_t tmp;
                load<uint8_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_str(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_STR_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_BIN_8: {
                uint8_t tmp;
                load<uint8_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_bin(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_BIN_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_EXT_8: {
                uint8_t tmp;
                load<uint8_t>(tmp, n);
                m_trail = tmp + 1;
                if(m_trail == 0) {
                    unpack_ext(m_user, n, m_trail, obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_EXT_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_STR_16: {
                uint16_t tmp;
                load<uint16_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_str(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_STR_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_BIN_16: {
                uint16_t tmp;
                load<uint16_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_bin(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_BIN_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_EXT_16: {
                uint16_t tmp;
                load<uint16_t>(tmp, n);
                m_trail = tmp + 1;
                if(m_trail == 0) {
                    unpack_ext(m_user, n, m_trail, obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_EXT_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_STR_32: {
                uint32_t tmp;
                load<uint32_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_str(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_STR_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_BIN_32: {
                uint32_t tmp;
                load<uint32_t>(tmp, n);
                m_trail = tmp;
                if(m_trail == 0) {
                    unpack_bin(m_user, n, static_cast<uint32_t>(m_trail), obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_BIN_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_CS_EXT_32: {
                uint32_t tmp;
                load<uint32_t>(tmp, n);
                check_ext_size<sizeof(std::size_t)>(tmp);
                m_trail = tmp;
                ++m_trail;
                if(m_trail == 0) {
                    unpack_ext(m_user, n, m_trail, obj);
                    int ret = push_proc(obj, off);
                    if (ret != 0) return ret;
                }
                else {
                    m_cs = MSGPACK_ACS_EXT_VALUE;
                    fixed_trail_again = true;
                }
            } break;
            case MSGPACK_ACS_STR_VALUE: {
                unpack_str(m_user, n, static_cast<uint32_t>(m_trail), obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_ACS_BIN_VALUE: {
                unpack_bin(m_user, n, static_cast<uint32_t>(m_trail), obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_ACS_EXT_VALUE: {
                unpack_ext(m_user, n, m_trail, obj);
                int ret = push_proc(obj, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_ARRAY_16: {
                int ret = push_aggregate<uint16_t>(
                    unpack_array(), MSGPACK_CT_ARRAY_ITEM, obj, n, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_ARRAY_32: {
                /* FIXME security guard */
                int ret = push_aggregate<uint32_t>(
                    unpack_array(), MSGPACK_CT_ARRAY_ITEM, obj, n, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_MAP_16: {
                int ret = push_aggregate<uint16_t>(
                    unpack_map(), MSGPACK_CT_MAP_KEY, obj, n, off);
                if (ret != 0) return ret;
            } break;
            case MSGPACK_CS_MAP_32: {
                /* FIXME security guard */
                int ret = push_aggregate<uint32_t>(
                    unpack_map(), MSGPACK_CT_MAP_KEY, obj, n, off);
                if (ret != 0) return ret;
            } break;
            default:
                off = m_current - m_start;
                return -1;
            }
        }
    } while(m_current != pe);

    off = m_current - m_start;
    return 0;
}

} // detail


typedef object_handle unpacked;

class unpacker {
public:
    unpacker(unpack_reference_func f = &unpacker::default_reference_func,
             void* user_data = nullptr,
             std::size_t init_buffer_size = MSGPACK_UNPACKER_INIT_BUFFER_SIZE,
             unpack_limit const& limit = unpack_limit());

#if !defined(MSGPACK_USE_CPP03)
    unpacker(unpacker&& other);
    unpacker& operator=(unpacker&& other);
#endif // !defined(MSGPACK_USE_CPP03)

    ~unpacker();

public:
    /*! 1. reserve buffer. at least `size' bytes of capacity will be ready */
    void reserve_buffer(std::size_t size = MSGPACK_UNPACKER_RESERVE_SIZE);

    /*! 2. read data to the buffer() up to buffer_capacity() bytes */
    char* buffer();
    std::size_t buffer_capacity() const;

    /*! 3. specify the number of bytes actually copied */
    void buffer_consumed(std::size_t size);

    /*! 4. repeat next() until it retunrs false */
    bool next(unpacked* result);
    bool next(unpacked& result, bool& referenced);
    bool next(unpacked& result);

    /*! 5. check if the size of message doesn't exceed assumption. */
    std::size_t message_size() const;

    // Basic usage of the unpacker is as following:
    //
    // unpacker pac;
    // while( /* input is readable */ ) {
    //
    //     // 1.
    //     pac.reserve_buffer(32*1024);
    //
    //     // 2.
    //     std::size_t bytes = input.readsome(pac.buffer(), pac.buffer_capacity());
    //
    //     // error handling ...
    //
    //     // 3.
    //     pac.buffer_consumed(bytes);
    //
    //     // 4.
    //     unpacked result;
    //     while(pac.next(&result)) {
    //         // do some with the object with the zone.
    //         object obj = result.get();
    //         std::auto_ptr<msgpack:zone> z = result.zone();
    //         on_message(obj, z);
    //
    //         //// boost::shared_ptr is also usable:
    //         // boost::shared_ptr<zone> life(z.release());
    //         // on_message(result.get(), life);
    //     }
    //
    //     // 5.
    //     if(pac.message_size() > 10*1024*1024) {
    //         throw std::runtime_error("message is too large");
    //     }
    // }
    //

    /*! for backward compatibility */
    bool execute();

    /*! for backward compatibility */
    clmdep_msgpack::object const& data();

    /*! for backward compatibility */
    clmdep_msgpack::zone* release_zone();

    /*! for backward compatibility */
    void reset_zone();

    /*! for backward compatibility */
    void reset();

public:
    // These functions are usable when non-MessagePack message follows after
    // MessagePack message.
    std::size_t parsed_size() const;

    /*! get address of the buffer that is not parsed */
    char* nonparsed_buffer();
    std::size_t nonparsed_size() const;

    /*! skip specified size of non-parsed buffer, leaving the buffer */
    // Note that the `size' argument must be smaller than nonparsed_size()
    void skip_nonparsed_buffer(std::size_t size);

    /*! remove unparsed buffer from unpacker */
    // Note that reset() leaves non-parsed buffer.
    void remove_nonparsed_buffer();

private:
    void expand_buffer(std::size_t size);
    int execute_imp();
    bool flush_zone();
    static bool default_reference_func(clmdep_msgpack::type::object_type type, std::size_t len, void*);

private:
    char* m_buffer;
    std::size_t m_used;
    std::size_t m_free;
    std::size_t m_off;
    std::size_t m_parsed;
    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> m_z;
    std::size_t m_initial_buffer_size;
    detail::context m_ctx;

#if defined(MSGPACK_USE_CPP03)
private:
    unpacker(const unpacker&);
    unpacker& operator=(const unpacker&);
#else  // defined(MSGPACK_USE_CPP03)
    unpacker(const unpacker&) = delete;
    unpacker& operator=(const unpacker&) = delete;
#endif // defined(MSGPACK_USE_CPP03)
};

unpacked unpack(
    const char* data, std::size_t len, std::size_t& off, bool& referenced,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
unpacked unpack(
    const char* data, std::size_t len, std::size_t& off,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
unpacked unpack(
    const char* data, std::size_t len, bool& referenced,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
unpacked unpack(
    const char* data, std::size_t len,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());


void unpack(unpacked& result,
            const char* data, std::size_t len, std::size_t& off, bool& referenced,
            unpack_reference_func f = nullptr, void* user_data = nullptr,
            unpack_limit const& limit = unpack_limit());
void unpack(unpacked& result,
            const char* data, std::size_t len, std::size_t& off,
            unpack_reference_func f = nullptr, void* user_data = nullptr,
            unpack_limit const& limit = unpack_limit());
void unpack(unpacked& result,
            const char* data, std::size_t len, bool& referenced,
            unpack_reference_func f = nullptr, void* user_data = nullptr,
            unpack_limit const& limit = unpack_limit());

void unpack(unpacked& result,
            const char* data, std::size_t len,
            unpack_reference_func f = nullptr, void* user_data = nullptr,
            unpack_limit const& limit = unpack_limit());

clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, std::size_t& off, bool& referenced,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, std::size_t& off,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, bool& referenced,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());
clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len,
    unpack_reference_func f = nullptr, void* user_data = nullptr,
    unpack_limit const& limit = unpack_limit());


// obsolete
void unpack(unpacked* result,
            const char* data, std::size_t len, std::size_t* off = nullptr, bool* referenced = nullptr,
            unpack_reference_func f = nullptr, void* user_data = nullptr,
            unpack_limit const& limit = unpack_limit());


// for internal use
typedef enum {
    UNPACK_SUCCESS              =  2,
    UNPACK_EXTRA_BYTES          =  1,
    UNPACK_CONTINUE             =  0,
    UNPACK_PARSE_ERROR          = -1
} unpack_return;

inline unpacker::unpacker(unpack_reference_func f,
                          void* user_data,
                          std::size_t initial_buffer_size,
                          unpack_limit const& limit)
    :m_z(new clmdep_msgpack::zone), m_ctx(f, user_data, limit)
{
    if(initial_buffer_size < COUNTER_SIZE) {
        initial_buffer_size = COUNTER_SIZE;
    }

    char* buffer = static_cast<char*>(::malloc(initial_buffer_size));
    if(!buffer) {
        throw std::bad_alloc();
    }

    m_buffer = buffer;
    m_used = COUNTER_SIZE;
    m_free = initial_buffer_size - m_used;
    m_off = COUNTER_SIZE;
    m_parsed = 0;
    m_initial_buffer_size = initial_buffer_size;

    detail::init_count(m_buffer);

    m_ctx.init();
    m_ctx.user().set_zone(*m_z);
    m_ctx.user().set_referenced(false);
}

#if !defined(MSGPACK_USE_CPP03)
// Move constructor and move assignment operator

inline unpacker::unpacker(unpacker&& other)
    :m_buffer(other.m_buffer),
     m_used(other.m_used),
     m_free(other.m_free),
     m_off(other.m_off),
     m_parsed(other.m_parsed),
     m_z(std::move(other.m_z)),
     m_initial_buffer_size(other.m_initial_buffer_size),
     m_ctx(other.m_ctx) {
    other.m_buffer = nullptr;
}

inline unpacker& unpacker::operator=(unpacker&& other) {
    this->~unpacker();
    new (this) unpacker(std::move(other));
    return *this;
}

#endif // !defined(MSGPACK_USE_CPP03)


inline unpacker::~unpacker()
{
    // These checks are required for move operations.
    if (m_buffer) detail::decr_count(m_buffer);
}


inline void unpacker::reserve_buffer(std::size_t size)
{
    if(m_free >= size) return;
    expand_buffer(size);
}

inline void unpacker::expand_buffer(std::size_t size)
{
    if(m_used == m_off && detail::get_count(m_buffer) == 1
        && !m_ctx.user().referenced()) {
        // rewind buffer
        m_free += m_used - COUNTER_SIZE;
        m_used = COUNTER_SIZE;
        m_off  = COUNTER_SIZE;

        if(m_free >= size) return;
    }

    if(m_off == COUNTER_SIZE) {
        std::size_t next_size = (m_used + m_free) * 2;    // include COUNTER_SIZE
        while(next_size < size + m_used) {
            std::size_t tmp_next_size = next_size * 2;
            if (tmp_next_size <= next_size) {
                next_size = size + m_used;
                break;
            }
            next_size = tmp_next_size;
        }

        char* tmp = static_cast<char*>(::realloc(m_buffer, next_size));
        if(!tmp) {
            throw std::bad_alloc();
        }

        m_buffer = tmp;
        m_free = next_size - m_used;

    } else {
        std::size_t next_size = m_initial_buffer_size;  // include COUNTER_SIZE
        std::size_t not_parsed = m_used - m_off;
        while(next_size < size + not_parsed + COUNTER_SIZE) {
            std::size_t tmp_next_size = next_size * 2;
            if (tmp_next_size <= next_size) {
                next_size = size + not_parsed + COUNTER_SIZE;
                break;
            }
            next_size = tmp_next_size;
        }

        char* tmp = static_cast<char*>(::malloc(next_size));
        if(!tmp) {
            throw std::bad_alloc();
        }

        detail::init_count(tmp);

        std::memcpy(tmp+COUNTER_SIZE, m_buffer + m_off, not_parsed);

        if(m_ctx.user().referenced()) {
            try {
                m_z->push_finalizer(&detail::decr_count, m_buffer);
            }
            catch (...) {
                ::free(tmp);
                throw;
            }
            m_ctx.user().set_referenced(false);
        } else {
            detail::decr_count(m_buffer);
        }

        m_buffer = tmp;
        m_used  = not_parsed + COUNTER_SIZE;
        m_free  = next_size - m_used;
        m_off   = COUNTER_SIZE;
    }
}

inline char* unpacker::buffer()
{
    return m_buffer + m_used;
}

inline std::size_t unpacker::buffer_capacity() const
{
    return m_free;
}

inline void unpacker::buffer_consumed(std::size_t size)
{
    m_used += size;
    m_free -= size;
}

inline bool unpacker::next(unpacked& result, bool& referenced)
{
    referenced = false;
    int ret = execute_imp();
    if(ret < 0) {
        throw clmdep_msgpack::parse_error("parse error");
    }

    if(ret == 0) {
        result.zone().reset();
        result.set(object());
        return false;

    } else {
        referenced = m_ctx.user().referenced();
        result.zone().reset( release_zone() );
        result.set(data());
        reset();
        return true;
    }
}

inline bool unpacker::next(unpacked& result)
{
    bool referenced;
    return next(result, referenced);
}

inline bool unpacker::next(unpacked* result)
{
    return next(*result);
}


inline bool unpacker::execute()
{
    int ret = execute_imp();
    if(ret < 0) {
        throw clmdep_msgpack::parse_error("parse error");
    } else if(ret == 0) {
        return false;
    } else {
        return true;
    }
}

inline int unpacker::execute_imp()
{
    std::size_t off = m_off;
    int ret = m_ctx.execute(m_buffer, m_used, m_off);
    if(m_off > off) {
        m_parsed += m_off - off;
    }
    return ret;
}

inline clmdep_msgpack::object const& unpacker::data()
{
    return m_ctx.data();
}

inline clmdep_msgpack::zone* unpacker::release_zone()
{
    if(!flush_zone()) {
        return nullptr;
    }

    clmdep_msgpack::zone* r =  new clmdep_msgpack::zone;
    clmdep_msgpack::zone* old = m_z.release();
    m_z.reset(r);
    m_ctx.user().set_zone(*m_z);

    return old;
}

inline void unpacker::reset_zone()
{
    m_z->clear();
}

inline bool unpacker::flush_zone()
{
    if(m_ctx.user().referenced()) {
        try {
            m_z->push_finalizer(&detail::decr_count, m_buffer);
        } catch (...) {
            return false;
        }
        m_ctx.user().set_referenced(false);

        detail::incr_count(m_buffer);
    }

    return true;
}

inline void unpacker::reset()
{
    m_ctx.init();
    // don't reset referenced flag
    m_parsed = 0;
}

inline std::size_t unpacker::message_size() const
{
    return m_parsed - m_off + m_used;
}

inline std::size_t unpacker::parsed_size() const
{
    return m_parsed;
}

inline char* unpacker::nonparsed_buffer()
{
    return m_buffer + m_off;
}

inline std::size_t unpacker::nonparsed_size() const
{
    return m_used - m_off;
}

inline void unpacker::skip_nonparsed_buffer(std::size_t size)
{
    m_off += size;
}

inline void unpacker::remove_nonparsed_buffer()
{
    m_used = m_off;
}

namespace detail {

inline unpack_return
unpack_imp(const char* data, std::size_t len, std::size_t& off,
           clmdep_msgpack::zone& result_zone, clmdep_msgpack::object& result, bool& referenced,
           unpack_reference_func f = nullptr, void* user_data = nullptr,
           unpack_limit const& limit = unpack_limit())
{
    std::size_t noff = off;

    if(len <= noff) {
        // FIXME
        return UNPACK_CONTINUE;
    }

    detail::context ctx(f, user_data, limit);
    ctx.init();

    ctx.user().set_zone(result_zone);
    ctx.user().set_referenced(false);
    referenced = false;

    int e = ctx.execute(data, len, noff);
    if(e < 0) {
        return UNPACK_PARSE_ERROR;
    }

    referenced = ctx.user().referenced();
    off = noff;

    if(e == 0) {
        return UNPACK_CONTINUE;
    }

    result = ctx.data();

    if(noff < len) {
        return UNPACK_EXTRA_BYTES;
    }

    return UNPACK_SUCCESS;
}

} // detail

// reference version

inline unpacked unpack(
    const char* data, std::size_t len, std::size_t& off, bool& referenced,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    clmdep_msgpack::object obj;
    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> z(new clmdep_msgpack::zone);
    referenced = false;
    std::size_t noff = off;
    unpack_return ret = detail::unpack_imp(
        data, len, noff, *z, obj, referenced, f, user_data, limit);

    switch(ret) {
    case UNPACK_SUCCESS:
        off = noff;
        return unpacked(obj, clmdep_msgpack::move(z));
    case UNPACK_EXTRA_BYTES:
        off = noff;
        return unpacked(obj, clmdep_msgpack::move(z));
    case UNPACK_CONTINUE:
        throw clmdep_msgpack::insufficient_bytes("insufficient bytes");
    case UNPACK_PARSE_ERROR:
    default:
        throw clmdep_msgpack::parse_error("parse error");
    }
    return unpacked();
}

inline unpacked unpack(
    const char* data, std::size_t len, std::size_t& off,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    return unpack(data, len, off, referenced, f, user_data, limit);
}

inline unpacked unpack(
    const char* data, std::size_t len, bool& referenced,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    std::size_t off = 0;
    return unpack(data, len, off, referenced, f, user_data, limit);
}

inline unpacked unpack(
    const char* data, std::size_t len,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    std::size_t off = 0;
    return unpack(data, len, off, referenced, f, user_data, limit);
}

inline void unpack(unpacked& result,
                   const char* data, std::size_t len, std::size_t& off, bool& referenced,
                   unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    clmdep_msgpack::object obj;
    clmdep_msgpack::unique_ptr<clmdep_msgpack::zone> z(new clmdep_msgpack::zone);
    referenced = false;
    std::size_t noff = off;
    unpack_return ret = detail::unpack_imp(
        data, len, noff, *z, obj, referenced, f, user_data, limit);

    switch(ret) {
    case UNPACK_SUCCESS:
        off = noff;
        result.set(obj);
        result.zone() = clmdep_msgpack::move(z);
        return;
    case UNPACK_EXTRA_BYTES:
        off = noff;
        result.set(obj);
        result.zone() = clmdep_msgpack::move(z);
        return;
    case UNPACK_CONTINUE:
        throw clmdep_msgpack::insufficient_bytes("insufficient bytes");
    case UNPACK_PARSE_ERROR:
    default:
        throw clmdep_msgpack::parse_error("parse error");
    }
}

inline void unpack(unpacked& result,
                   const char* data, std::size_t len, std::size_t& off,
                   unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    unpack(result, data, len, off, referenced, f, user_data, limit);
}

inline void unpack(unpacked& result,
                   const char* data, std::size_t len, bool& referenced,
                   unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    std::size_t off = 0;
    unpack(result, data, len, off, referenced, f, user_data, limit);
}

inline void unpack(unpacked& result,
                   const char* data, std::size_t len,
                   unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    std::size_t off = 0;
    unpack(result, data, len, off, referenced, f, user_data, limit);
}


inline clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, std::size_t& off, bool& referenced,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    clmdep_msgpack::object obj;
    std::size_t noff = off;
    referenced = false;
    unpack_return ret = detail::unpack_imp(
        data, len, noff, z, obj, referenced, f, user_data, limit);

    switch(ret) {
    case UNPACK_SUCCESS:
        off = noff;
        return obj;
    case UNPACK_EXTRA_BYTES:
        off = noff;
        return obj;
    case UNPACK_CONTINUE:
        throw clmdep_msgpack::insufficient_bytes("insufficient bytes");
    case UNPACK_PARSE_ERROR:
    default:
        throw clmdep_msgpack::parse_error("parse error");
    }
    return obj;
}

inline clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, std::size_t& off,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    return unpack(z, data, len, off, referenced, f, user_data, limit);
}

inline clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len, bool& referenced,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    std::size_t off = 0;
    return unpack(z, data, len, off, referenced, f, user_data, limit);
}

inline clmdep_msgpack::object unpack(
    clmdep_msgpack::zone& z,
    const char* data, std::size_t len,
    unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    bool referenced;
    std::size_t off = 0;
    return unpack(z, data, len, off, referenced, f, user_data, limit);
}

// obsolete
// pointer version
inline void unpack(unpacked* result,
                   const char* data, std::size_t len, std::size_t* off, bool* referenced,
                   unpack_reference_func f, void* user_data, unpack_limit const& limit)
{
    if (off)
        if (referenced) unpack(*result, data, len, *off, *referenced, f, user_data, limit);
        else unpack(*result, data, len, *off, f, user_data, limit);
    else
        if (referenced) unpack(*result, data, len, *referenced, f, user_data, limit);
        else unpack(*result, data, len, f, user_data, limit);
}

inline bool unpacker::default_reference_func(clmdep_msgpack::type::object_type /*type*/, std::size_t /*len*/, void*)
{
    return true;
}

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack


#endif /* msgpack/unpack.hpp */
