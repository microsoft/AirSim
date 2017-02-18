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
#ifndef MSGPACK_TYPE_INT_HPP
#define MSGPACK_TYPE_INT_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include <limits>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1){
/// @endcond

namespace type {
namespace detail {
    template <typename T, bool Signed>
    struct convert_integer_sign;

    template <typename T>
    struct convert_integer_sign<T, true> {
        static inline T convert(clmdep_msgpack::object const& o) {
            if(o.type == clmdep_msgpack::type::POSITIVE_INTEGER) {
                if(o.via.u64 > static_cast<uint64_t>(std::numeric_limits<T>::max()))
                    { throw clmdep_msgpack::type_error(); }
                return static_cast<T>(o.via.u64);
            } else if(o.type == clmdep_msgpack::type::NEGATIVE_INTEGER) {
                if(o.via.i64 < static_cast<int64_t>(std::numeric_limits<T>::min()))
                    { throw clmdep_msgpack::type_error(); }
                return static_cast<T>(o.via.i64);
            }
            throw clmdep_msgpack::type_error();
        }
    };

    template <typename T>
    struct convert_integer_sign<T, false> {
        static inline T convert(clmdep_msgpack::object const& o) {
            if(o.type == clmdep_msgpack::type::POSITIVE_INTEGER) {
                if(o.via.u64 > static_cast<uint64_t>(std::numeric_limits<T>::max()))
                    { throw clmdep_msgpack::type_error(); }
                return static_cast<T>(o.via.u64);
            }
            throw clmdep_msgpack::type_error();
        }
    };

    template <typename T>
    struct is_signed {
        static const bool value = std::numeric_limits<T>::is_signed;
    };

    template <typename T>
    static inline T convert_integer(clmdep_msgpack::object const& o)
    {
        return detail::convert_integer_sign<T, is_signed<T>::value>::convert(o);
    }

    template <bool Signed>
    struct object_char_sign;

    template <>
    struct object_char_sign<true> {
        static inline void make(clmdep_msgpack::object& o, char v) {
            if (v < 0) {
                o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
                o.via.i64 = v;
            }
            else {
                o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
                o.via.u64 = v;
            }
        }
    };

    template <>
    struct object_char_sign<false> {
        static inline void make(clmdep_msgpack::object& o, char v) {
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v;
        }
    };

    static inline void object_char(clmdep_msgpack::object& o, char v) {
        return object_char_sign<is_signed<char>::value>::make(o, v);
    }

}  // namespace detail
}  // namespace type

namespace adaptor {

template <>
struct convert<char> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, char& v) const
    { v = type::detail::convert_integer<char>(o); return o; }
};

template <>
struct convert<signed char> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, signed char& v) const
    { v = type::detail::convert_integer<signed char>(o); return o; }
};

template <>
struct convert<signed short> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, signed short& v) const
    { v = type::detail::convert_integer<signed short>(o); return o; }
};

template <>
struct convert<signed int> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, signed int& v) const
    { v = type::detail::convert_integer<signed int>(o); return o; }
};

template <>
struct convert<signed long> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, signed long& v) const
    { v = type::detail::convert_integer<signed long>(o); return o; }
};

template <>
struct convert<signed long long> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, signed long long& v) const
    { v = type::detail::convert_integer<signed long long>(o); return o; }
};


template <>
struct convert<unsigned char> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, unsigned char& v) const
    { v = type::detail::convert_integer<unsigned char>(o); return o; }
};

template <>
struct convert<unsigned short> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, unsigned short& v) const
    { v = type::detail::convert_integer<unsigned short>(o); return o; }
};

template <>
struct convert<unsigned int> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, unsigned int& v) const
    { v = type::detail::convert_integer<unsigned int>(o); return o; }
};

template <>
struct convert<unsigned long> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, unsigned long& v) const
    { v = type::detail::convert_integer<unsigned long>(o); return o; }
};

template <>
struct convert<unsigned long long> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, unsigned long long& v) const
    { v = type::detail::convert_integer<unsigned long long>(o); return o; }
};


template <>
struct pack<char> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, char v) const
    { o.pack_char(v); return o; }
};

template <>
struct pack<signed char> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, signed char v) const
    { o.pack_signed_char(v); return o; }
};

template <>
struct pack<signed short> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, signed short v) const
    { o.pack_short(v); return o; }
};

template <>
struct pack<signed int> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, signed int v) const
    { o.pack_int(v); return o; }
};

template <>
struct pack<signed long> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, signed long v) const
    { o.pack_long(v); return o; }
};

template <>
struct pack<signed long long> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, signed long long v) const
    { o.pack_long_long(v); return o; }
};


template <>
struct pack<unsigned char> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, unsigned char v) const
    { o.pack_unsigned_char(v); return o; }
};

template <>
struct pack<unsigned short> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, unsigned short v) const
    { o.pack_unsigned_short(v); return o; }
};

template <>
struct pack<unsigned int> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, unsigned int v) const
    { o.pack_unsigned_int(v); return o; }
};

template <>
struct pack<unsigned long> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, unsigned long v) const
    { o.pack_unsigned_long(v); return o; }
};

template <>
struct pack<unsigned long long> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, unsigned long long v) const
    { o.pack_unsigned_long_long(v); return o; }
};


template <>
struct object<char> {
    void operator()(clmdep_msgpack::object& o, char v) const
    { type::detail::object_char(o, v); }
};

template <>
struct object<signed char> {
    void operator()(clmdep_msgpack::object& o, signed char v) const {
        if (v < 0) {
            o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
            o.via.i64 = v;
        }
        else {
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
            o.via.u64 = v;
        }
    }
};

template <>
struct object<signed short> {
    void operator()(clmdep_msgpack::object& o, signed short v) const {
        if (v < 0) {
            o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
            o.via.i64 = v;
        }
        else {
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
            o.via.u64 = v;
        }
    }
};

template <>
struct object<signed int> {
    void operator()(clmdep_msgpack::object& o, signed int v) const {
        if (v < 0) {
            o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
            o.via.i64 = v;
        }
        else {
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
            o.via.u64 = v;
        }
    }
};

template <>
struct object<signed long> {
    void operator()(clmdep_msgpack::object& o, signed long v) const {
        if (v < 0) {
            o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
            o.via.i64 = v;
        }
        else {
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
            o.via.u64 = v;
        }
    }
};

template <>
struct object<signed long long> {
    void operator()(clmdep_msgpack::object& o, signed long long v) const {
        if (v < 0) {
            o.type = clmdep_msgpack::type::NEGATIVE_INTEGER;
            o.via.i64 = v;
        }
        else{
            o.type = clmdep_msgpack::type::POSITIVE_INTEGER;
            o.via.u64 = v;
        }
    }
};

template <>
struct object<unsigned char> {
    void operator()(clmdep_msgpack::object& o, unsigned char v) const
    { o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v; }
};

template <>
struct object<unsigned short> {
    void operator()(clmdep_msgpack::object& o, unsigned short v) const
    { o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v; }
};

template <>
struct object<unsigned int> {
    void operator()(clmdep_msgpack::object& o, unsigned int v) const
    { o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v; }
};

template <>
struct object<unsigned long> {
    void operator()(clmdep_msgpack::object& o, unsigned long v) const
    { o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v; }
};

template <>
struct object<unsigned long long> {
    void operator()(clmdep_msgpack::object& o, unsigned long long v) const
    { o.type = clmdep_msgpack::type::POSITIVE_INTEGER, o.via.u64 = v; }
};


template <>
struct object_with_zone<char> {
    void operator()(clmdep_msgpack::object::with_zone& o, char v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<signed char> {
    void operator()(clmdep_msgpack::object::with_zone& o, signed char v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<signed short> {
    void operator()(clmdep_msgpack::object::with_zone& o, signed short v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<signed int> {
    void operator()(clmdep_msgpack::object::with_zone& o, signed int v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<signed long> {
    void operator()(clmdep_msgpack::object::with_zone& o, signed long v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<signed long long> {
    void operator()(clmdep_msgpack::object::with_zone& o, const signed long long& v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<unsigned char> {
    void operator()(clmdep_msgpack::object::with_zone& o, unsigned char v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<unsigned short> {
    void operator()(clmdep_msgpack::object::with_zone& o, unsigned short v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<unsigned int> {
    void operator()(clmdep_msgpack::object::with_zone& o, unsigned int v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<unsigned long> {
    void operator()(clmdep_msgpack::object::with_zone& o, unsigned long v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

template <>
struct object_with_zone<unsigned long long> {
    void operator()(clmdep_msgpack::object::with_zone& o, const unsigned long long& v) const
    { static_cast<clmdep_msgpack::object&>(o) << v; }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif /* msgpack/type/int.hpp */
