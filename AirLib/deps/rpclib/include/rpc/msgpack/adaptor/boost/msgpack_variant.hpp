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
#ifndef MSGPACK_TYPE_BOOST_MSGPACK_VARIANT_HPP
#define MSGPACK_TYPE_BOOST_MSGPACK_VARIANT_HPP

#if defined(MSGPACK_USE_BOOST)

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"
#include "rpc/msgpack/adaptor/boost/string_ref.hpp"

#include "rpc/msgpack/adaptor/nil.hpp"
#include "rpc/msgpack/adaptor/bool.hpp"
#include "rpc/msgpack/adaptor/int.hpp"
#include "rpc/msgpack/adaptor/float.hpp"
#include "rpc/msgpack/adaptor/string.hpp"
#include "rpc/msgpack/adaptor/vector_char.hpp"
#include "rpc/msgpack/adaptor/raw.hpp"
#include "rpc/msgpack/adaptor/ext.hpp"
#include "rpc/msgpack/adaptor/vector.hpp"
#include "rpc/msgpack/adaptor/map.hpp"

#include <boost/variant.hpp>
#include <boost/operators.hpp>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {


template <typename STR, typename BIN, typename EXT>
struct basic_variant :
    boost::variant<
        nil,               // NIL
        bool,              // BOOL
        int64_t,           // NEGATIVE_INTEGER
        uint64_t,          // POSITIVE_INTEGER
        double,            // FLOAT
        std::string,       // STR
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
        boost::string_ref, // STR
#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
        std::vector<char>, // BIN
        clmdep_msgpack::type::raw_ref, // BIN
        ext,               // EXT
        ext_ref,           // EXT
        boost::recursive_wrapper<std::vector<basic_variant<STR, BIN, EXT> > >, // ARRAY
        boost::recursive_wrapper<std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >, // MAP
        boost::recursive_wrapper<std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >// MAP
    >,
    private boost::totally_ordered<basic_variant<STR, BIN, EXT> > {
    typedef boost::variant<
        nil,               // NIL
        bool,              // BOOL
        int64_t,           // NEGATIVE_INTEGER
        uint64_t,          // POSITIVE_INTEGER
        double,            // FLOAT
        std::string,       // STR
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
        boost::string_ref, // STR
#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
        std::vector<char>, // BIN
        clmdep_msgpack::type::raw_ref, // BIN
        ext,               // EXT
        ext_ref,           // EXT
        boost::recursive_wrapper<std::vector<basic_variant<STR, BIN, EXT> > >, // ARRAY
        boost::recursive_wrapper<std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >, // MAP
        boost::recursive_wrapper<std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >// MAP
    > base;
    basic_variant() {}
    template <typename T>
    basic_variant(T const& t):base(t) {}
    basic_variant(char const* p):base(std::string(p)) {}
    basic_variant(char v) {
        int_init(v);
    }
    basic_variant(signed char v) {
        int_init(v);
    }
    basic_variant(unsigned char v):base(uint64_t(v)) {}
    basic_variant(signed int v) {
        int_init(v);
    }
    basic_variant(unsigned int v):base(uint64_t(v)) {}
    basic_variant(signed long v) {
        int_init(v);
    }
    basic_variant(unsigned long v):base(uint64_t(v)) {}
    basic_variant(signed long long v) {
        int_init(v);
    }
    basic_variant(unsigned long long v):base(uint64_t(v)) {}

    bool is_nil() const {
        return boost::get<nil>(this);
    }
    bool is_bool() const {
        return boost::get<bool>(this);
    }
    bool is_int64_t() const {
        return boost::get<int64_t>(this);
    }
    bool is_uint64_t() const {
        return boost::get<uint64_t>(this);
    }
    bool is_double() const {
        return boost::get<double>(this);
    }
    bool is_string() const {
        return boost::get<std::string>(this);
    }
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    bool is_boost_string_ref() const {
        return boost::get<boost::string_ref>(this);
    }
#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    bool is_vector_char() const {
        return boost::get<std::vector<char> >(this);
    }
    bool is_vector_char() {
        return boost::get<std::vector<char> >(this);
    }
    bool is_raw_ref() const {
        return boost::get<raw_ref>(this);
    }
    bool is_ext() const {
        return boost::get<ext>(this);
    }
    bool is_ext_ref() const {
        return boost::get<ext_ref>(this);
    }
    bool is_vector() const {
        return boost::get<std::vector<basic_variant<STR, BIN, EXT> > >(this);
    }
    bool is_map() const {
        return boost::get<std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(this);
    }
    bool is_multimap() const {
        return boost::get<std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(this);
    }

    bool as_bool() const {
        return boost::get<bool>(*this);
    }
    int64_t as_int64_t() const {
        return boost::get<int64_t>(*this);
    }
    int64_t& as_int64_t() {
        return boost::get<int64_t>(*this);
    }
    uint64_t as_uint64_t() const {
        return boost::get<uint64_t>(*this);
    }
    uint64_t& as_uint64_t() {
        return boost::get<uint64_t>(*this);
    }
    double as_double() const {
        return boost::get<double>(*this);
    }
    double& as_double() {
        return boost::get<double>(*this);
    }
    std::string const& as_string() const {
        return boost::get<std::string>(*this);
    }
    std::string& as_string() {
        return boost::get<std::string>(*this);
    }
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    boost::string_ref const& as_boost_string_ref() const {
        return boost::get<boost::string_ref>(*this);
    }
    boost::string_ref& as_boost_string_ref() {
        return boost::get<boost::string_ref>(*this);
    }
#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    std::vector<char> const& as_vector_char() const {
        return boost::get<std::vector<char> >(*this);
    }
    std::vector<char>& as_vector_char() {
        return boost::get<std::vector<char> >(*this);
    }
    raw_ref const& as_raw_ref() const {
        return boost::get<raw_ref>(*this);
    }
    ext const& as_ext() const {
        return boost::get<ext>(*this);
    }
    ext& as_ext() {
        return boost::get<ext>(*this);
    }
    ext_ref const& as_ext_ref() const {
        return boost::get<ext_ref>(*this);
    }
    std::vector<basic_variant<STR, BIN, EXT> > const& as_vector() const {
        return boost::get<std::vector<basic_variant<STR, BIN, EXT> > >(*this);
    }
    std::vector<basic_variant<STR, BIN, EXT> >& as_vector() {
        return boost::get<std::vector<basic_variant<STR, BIN, EXT> > >(*this);
    }
    std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > const& as_map() const {
        return boost::get<std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(*this);
    }
    std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> >& as_map() {
        return boost::get<std::map<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(*this);
    }
    std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > const& as_multimap() const {
        return boost::get<std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(*this);
    }
    std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> >& as_multimap() {
        return boost::get<std::multimap<basic_variant<STR, BIN, EXT>, basic_variant<STR, BIN, EXT> > >(*this);
    }
private:
    template <typename T>
    void int_init(T v) {
        if (v < 0) {
            static_cast<base&>(*this) = int64_t(v);
        }
        else {
            static_cast<base&>(*this) = uint64_t(v);
        }
    }
};

template <typename STR, typename BIN, typename EXT>
inline bool operator<(basic_variant<STR, BIN, EXT> const& lhs, basic_variant<STR, BIN, EXT> const& rhs) {
    return
        static_cast<typename basic_variant<STR, BIN, EXT>::base const&>(lhs) <
        static_cast<typename basic_variant<STR, BIN, EXT>::base const&>(rhs);
}

template <typename STR, typename BIN, typename EXT>
inline bool operator==(basic_variant<STR, BIN, EXT> const& lhs, basic_variant<STR, BIN, EXT> const& rhs) {
    return
        static_cast<typename basic_variant<STR, BIN, EXT>::base const&>(lhs) ==
        static_cast<typename basic_variant<STR, BIN, EXT>::base const&>(rhs);
}

typedef basic_variant<std::string, std::vector<char>, ext> variant;
typedef basic_variant<
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    boost::string_ref,
#else  // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    std::string,
#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53
    raw_ref, ext_ref> variant_ref;

} // namespace type

namespace adaptor {

#if !defined (MSGPACK_USE_CPP03)

template <typename STR, typename BIN, typename EXT>
struct as<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > {
    clmdep_msgpack::type::basic_variant<STR, BIN, EXT> operator()(clmdep_msgpack::object const& o) const {
        switch(o.type) {
        case type::NIL:
            return o.as<clmdep_msgpack::type::nil>();
        case type::BOOLEAN:
            return o.as<bool>();
        case type::POSITIVE_INTEGER:
            return o.as<uint64_t>();
        case type::NEGATIVE_INTEGER:
            return o.as<int64_t>();
        case type::FLOAT:
            return o.as<double>();
        case type::STR:
            return o.as<STR>();
        case type::BIN:
            return o.as<BIN>();
        case type::EXT:
            return o.as<EXT>();
        case type::ARRAY:
            return o.as<std::vector<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > >();
        case type::MAP:
            return o.as<std::multimap<clmdep_msgpack::type::basic_variant<STR, BIN, EXT>, clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > >();
        default:
            break;
        }
        return clmdep_msgpack::type::basic_variant<STR, BIN, EXT>();
    }
};

#endif // !defined (MSGPACK_USE_CPP03)


template <typename STR, typename BIN, typename EXT>
struct convert<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > {
    clmdep_msgpack::object const& operator()(
        clmdep_msgpack::object const& o,
        clmdep_msgpack::type::basic_variant<STR, BIN, EXT>& v) const {
        switch(o.type) {
        case type::NIL:
            v = o.as<clmdep_msgpack::type::nil>();
            break;
        case type::BOOLEAN:
            v = o.as<bool>();
            break;
        case type::POSITIVE_INTEGER:
            v = o.as<uint64_t>();
            break;
        case type::NEGATIVE_INTEGER:
            v = o.as<int64_t>();
            break;
        case type::FLOAT:
            v = o.as<double>();
            break;
        case type::STR:
            v = o.as<STR>();
            break;
        case type::BIN:
            v = o.as<BIN>();
            break;
        case type::EXT:
            v = o.as<EXT>();
            break;
        case type::ARRAY:
            v = o.as<std::vector<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > >();
            break;
        case type::MAP:
            v = o.as<std::multimap<clmdep_msgpack::type::basic_variant<STR, BIN, EXT>, clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > >();
            break;
        default:
            break;
        }
        return o;
    }
};

namespace detail {

template <typename Stream>
struct pack_imp : boost::static_visitor<void> {
    template <typename T>
    void operator()(T const& value) const {
        pack<T>()(o_, value);
    }
    pack_imp(packer<Stream>& o):o_(o) {}
    packer<Stream>& o_;
};

} // namespace detail

template <typename STR, typename BIN, typename EXT>
struct pack<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const clmdep_msgpack::type::basic_variant<STR, BIN, EXT>& v) const {
        boost::apply_visitor(detail::pack_imp<Stream>(o), v);
        return o;
    }
};

namespace detail {

struct object_imp : boost::static_visitor<void> {
    void operator()(clmdep_msgpack::type::nil const& v) const {
        object<clmdep_msgpack::type::nil>()(o_, v);
    }
    void operator()(bool const& v) const {
        object<bool>()(o_, v);
    }
    void operator()(uint64_t const& v) const {
        object<uint64_t>()(o_, v);
    }
    void operator()(int64_t const& v) const {
        object<int64_t>()(o_, v);
    }
    void operator()(double const& v) const {
        object<double>()(o_, v);
    }
    template <typename T>
    void operator()(T const&) const {
        throw clmdep_msgpack::type_error();
    }
    object_imp(clmdep_msgpack::object& o):o_(o) {}
    clmdep_msgpack::object& o_;
};

} // namespace detail

template <typename STR, typename BIN, typename EXT>
struct object<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > {
    void operator()(clmdep_msgpack::object& o, const clmdep_msgpack::type::basic_variant<STR, BIN, EXT>& v) const {
        boost::apply_visitor(detail::object_imp(o), v);
    }
};

namespace detail {

struct object_with_zone_imp : boost::static_visitor<void> {
    template <typename T>
    void operator()(T const& v) const {
        object_with_zone<T>()(o_, v);
    }
    object_with_zone_imp(clmdep_msgpack::object::with_zone& o):o_(o) {}
    clmdep_msgpack::object::with_zone& o_;
};

} // namespace detail

template <typename STR, typename BIN, typename EXT>
struct object_with_zone<clmdep_msgpack::type::basic_variant<STR, BIN, EXT> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const clmdep_msgpack::type::basic_variant<STR, BIN, EXT>& v) const {
        boost::apply_visitor(detail::object_with_zone_imp(o), v);
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_USE_BOOST
#endif // MSGPACK_TYPE_BOOST_MSGPACK_VARIANT_HPP
