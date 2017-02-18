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
#ifndef MSGPACK_CPP11_TUPLE_HPP
#define MSGPACK_CPP11_TUPLE_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"
#include "rpc/msgpack/meta.hpp"

#include <tuple>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

// --- Pack from tuple to packer stream ---
template <typename Stream, typename Tuple, std::size_t N>
struct StdTuplePacker {
    static void pack(
        clmdep_msgpack::packer<Stream>& o,
        const Tuple& v) {
        StdTuplePacker<Stream, Tuple, N-1>::pack(o, v);
        o.pack(std::get<N-1>(v));
    }
};

template <typename Stream, typename Tuple>
struct StdTuplePacker<Stream, Tuple, 0> {
    static void pack (
        clmdep_msgpack::packer<Stream>&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct pack<std::tuple<Args...>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(
        clmdep_msgpack::packer<Stream>& o,
        const std::tuple<Args...>& v) const {
        uint32_t size = checked_get_container_size(sizeof...(Args));
        o.pack_array(size);
        StdTuplePacker<Stream, decltype(v), sizeof...(Args)>::pack(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object ---

template <typename... Args>
struct StdTupleAs;

template <typename T, typename... Args>
struct StdTupleAsImpl {
    static std::tuple<T, Args...> as(clmdep_msgpack::object const& o) {
        return std::tuple_cat(
            std::make_tuple(o.via.array.ptr[o.via.array.size - sizeof...(Args) - 1].as<T>()),
            StdTupleAs<Args...>::as(o));
    }
};

template <typename... Args>
struct StdTupleAs {
    static std::tuple<Args...> as(clmdep_msgpack::object const& o) {
        return StdTupleAsImpl<Args...>::as(o);
    }
};

template <>
struct StdTupleAs<> {
    static std::tuple<> as (clmdep_msgpack::object const&) {
        return std::tuple<>();
    }
};

template <typename Tuple, std::size_t N>
struct StdTupleConverter {
    static void convert(
        clmdep_msgpack::object const& o,
        Tuple& v) {
        StdTupleConverter<Tuple, N-1>::convert(o, v);
        o.via.array.ptr[N-1].convert<typename std::remove_reference<decltype(std::get<N-1>(v))>::type>(std::get<N-1>(v));
    }
};

template <typename Tuple>
struct StdTupleConverter<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object const&,
        Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct as<std::tuple<Args...>, typename std::enable_if<clmdep_msgpack::all_of<clmdep_msgpack::has_as, Args...>::value>::type>  {
    std::tuple<Args...> operator()(
        clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if (o.via.array.size < sizeof...(Args)) { throw clmdep_msgpack::type_error(); }
        return StdTupleAs<Args...>::as(o);
    }
};

template <typename... Args>
struct convert<std::tuple<Args...>> {
    clmdep_msgpack::object const& operator()(
        clmdep_msgpack::object const& o,
        std::tuple<Args...>& v) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if(o.via.array.size < sizeof...(Args)) { throw clmdep_msgpack::type_error(); }
        StdTupleConverter<decltype(v), sizeof...(Args)>::convert(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object with zone ---
template <typename Tuple, std::size_t N>
struct StdTupleToObjectWithZone {
    static void convert(
        clmdep_msgpack::object::with_zone& o,
        const Tuple& v) {
        StdTupleToObjectWithZone<Tuple, N-1>::convert(o, v);
        o.via.array.ptr[N-1] = clmdep_msgpack::object(std::get<N-1>(v), o.zone);
    }
};

template <typename Tuple>
struct StdTupleToObjectWithZone<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object::with_zone&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct object_with_zone<std::tuple<Args...>> {
    void operator()(
        clmdep_msgpack::object::with_zone& o,
        std::tuple<Args...> const& v) const {
        uint32_t size = checked_get_container_size(sizeof...(Args));
        o.type = clmdep_msgpack::type::ARRAY;
        o.via.array.ptr = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size));
        o.via.array.size = size;
        StdTupleToObjectWithZone<decltype(v), sizeof...(Args)>::convert(o, v);
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_CPP11_TUPLE_HPP
