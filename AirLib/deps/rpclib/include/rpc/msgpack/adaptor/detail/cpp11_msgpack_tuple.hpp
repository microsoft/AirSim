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
#ifndef MSGPACK_CPP11_MSGPACK_TUPLE_HPP
#define MSGPACK_CPP11_MSGPACK_TUPLE_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/object_fwd.hpp"
#include "rpc/msgpack/meta.hpp"

#include <tuple>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {
    // tuple
    using std::get;
    using std::tuple_size;
    using std::tuple_element;
    using std::uses_allocator;
    using std::ignore;
    using std::swap;

    template< class... Types >
    class tuple : public std::tuple<Types...> {
    public:
        using base = std::tuple<Types...>;

        using base::base;

        tuple() = default;
        tuple(tuple const&) = default;
        tuple(tuple&&) = default;

        template<typename... OtherTypes>
        tuple(tuple<OtherTypes...> const& other):base(static_cast<std::tuple<OtherTypes...> const&>(other)) {}
        template<typename... OtherTypes>
        tuple(tuple<OtherTypes...> && other):base(static_cast<std::tuple<OtherTypes...> &&>(other)) {}

        tuple& operator=(tuple const&) = default;
        tuple& operator=(tuple&&) = default;

        template<typename... OtherTypes>
        tuple& operator=(tuple<OtherTypes...> const& other) {
            *static_cast<base*>(this) = static_cast<std::tuple<OtherTypes...> const&>(other);
            return *this;
        }
        template<typename... OtherTypes>
        tuple& operator=(tuple<OtherTypes...> && other) {
            *static_cast<base*>(this) = static_cast<std::tuple<OtherTypes...> &&>(other);
            return *this;
        }

        template< std::size_t I>
        typename tuple_element<I, base >::type&
        get() & { return std::get<I>(*this); }

        template< std::size_t I>
        typename tuple_element<I, base >::type const&
        get() const& { return std::get<I>(*this); }

        template< std::size_t I>
        typename tuple_element<I, base >::type&&
        get() && { return std::get<I>(*this); }
    };

    template <class... Args>
    inline tuple<Args...> make_tuple(Args&&... args) {
        return tuple<Args...>(args...);
    }

    template<class... Args>
    inline tuple<Args&&...> forward_as_tuple (Args&&... args) noexcept {
        return tuple<Args&&...>(std::forward<Args>(args)...);
    }

    template <class... Tuples>
    inline auto tuple_cat(Tuples&&... args) ->
        decltype(
            std::tuple_cat(std::forward<typename std::remove_reference<Tuples>::type::base>(args)...)
        ) {
        return std::tuple_cat(std::forward<typename std::remove_reference<Tuples>::type::base>(args)...);
    }
    template <class... Args>
    inline tuple<Args&...> tie(Args&... args) {
        return tuple<Args&...>(args...);
    }
} // namespace type

// --- Pack from tuple to packer stream ---
template <typename Stream, typename Tuple, std::size_t N>
struct MsgpackTuplePacker {
    static void pack(
        clmdep_msgpack::packer<Stream>& o,
        const Tuple& v) {
        MsgpackTuplePacker<Stream, Tuple, N-1>::pack(o, v);
        o.pack(type::get<N-1>(v));
    }
};

template <typename Stream, typename Tuple>
struct MsgpackTuplePacker<Stream, Tuple, 1> {
    static void pack (
        clmdep_msgpack::packer<Stream>& o,
        const Tuple& v) {
        o.pack(type::get<0>(v));
    }
};

template <typename Stream, typename Tuple>
struct MsgpackTuplePacker<Stream, Tuple, 0> {
    static void pack (
        clmdep_msgpack::packer<Stream>&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct pack<clmdep_msgpack::type::tuple<Args...>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(
        clmdep_msgpack::packer<Stream>& o,
        const clmdep_msgpack::type::tuple<Args...>& v) const {
        o.pack_array(sizeof...(Args));
        MsgpackTuplePacker<Stream, decltype(v), sizeof...(Args)>::pack(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object ---

template <typename... Args>
struct MsgpackTupleAs;

template <typename T, typename... Args>
struct MsgpackTupleAsImpl {
    static clmdep_msgpack::type::tuple<T, Args...> as(clmdep_msgpack::object const& o) {
        return clmdep_msgpack::type::tuple_cat(
            clmdep_msgpack::type::make_tuple(o.via.array.ptr[o.via.array.size - sizeof...(Args) - 1].as<T>()),
            MsgpackTupleAs<Args...>::as(o));
    }
};

template <typename... Args>
struct MsgpackTupleAs {
    static clmdep_msgpack::type::tuple<Args...> as(clmdep_msgpack::object const& o) {
        return MsgpackTupleAsImpl<Args...>::as(o);
    }
};

template <>
struct MsgpackTupleAs<> {
    static clmdep_msgpack::type::tuple<> as (clmdep_msgpack::object const&) {
        return clmdep_msgpack::type::tuple<>();
    }
};

template <typename Tuple, std::size_t N>
struct MsgpackTupleConverter {
    static void convert(
        clmdep_msgpack::object const& o,
        Tuple& v) {
        MsgpackTupleConverter<Tuple, N-1>::convert(o, v);
        o.via.array.ptr[N-1].convert<typename std::remove_reference<decltype(type::get<N-1>(v))>::type>(type::get<N-1>(v));
    }
};

template <typename Tuple>
struct MsgpackTupleConverter<Tuple, 1> {
    static void convert (
        clmdep_msgpack::object const& o,
        Tuple& v) {
        o.via.array.ptr[0].convert<typename std::remove_reference<decltype(type::get<0>(v))>::type>(type::get<0>(v));
    }
};

template <typename Tuple>
struct MsgpackTupleConverter<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object const&,
        Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct as<clmdep_msgpack::type::tuple<Args...>, typename std::enable_if<clmdep_msgpack::all_of<clmdep_msgpack::has_as, Args...>::value>::type>  {
    clmdep_msgpack::type::tuple<Args...> operator()(
        clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if (o.via.array.size < sizeof...(Args)) { throw clmdep_msgpack::type_error(); }
        return MsgpackTupleAs<Args...>::as(o);
    }
};

template <typename... Args>
struct convert<clmdep_msgpack::type::tuple<Args...>> {
    clmdep_msgpack::object const& operator()(
        clmdep_msgpack::object const& o,
        clmdep_msgpack::type::tuple<Args...>& v) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if(o.via.array.size < sizeof...(Args)) { throw clmdep_msgpack::type_error(); }
        MsgpackTupleConverter<decltype(v), sizeof...(Args)>::convert(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object with zone ---
template <typename Tuple, std::size_t N>
struct MsgpackTupleToObjectWithZone {
    static void convert(
        clmdep_msgpack::object::with_zone& o,
        const Tuple& v) {
        MsgpackTupleToObjectWithZone<Tuple, N-1>::convert(o, v);
        o.via.array.ptr[N-1] = clmdep_msgpack::object(type::get<N-1>(v), o.zone);
    }
};

template <typename Tuple>
struct MsgpackTupleToObjectWithZone<Tuple, 1> {
    static void convert (
        clmdep_msgpack::object::with_zone& o,
        const Tuple& v) {
        o.via.array.ptr[0] = clmdep_msgpack::object(type::get<0>(v), o.zone);
    }
};

template <typename Tuple>
struct MsgpackTupleToObjectWithZone<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object::with_zone&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
    struct object_with_zone<clmdep_msgpack::type::tuple<Args...>> {
    void operator()(
        clmdep_msgpack::object::with_zone& o,
        clmdep_msgpack::type::tuple<Args...> const& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        o.via.array.ptr = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*sizeof...(Args)));
        o.via.array.size = sizeof...(Args);
        MsgpackTupleToObjectWithZone<decltype(v), sizeof...(Args)>::convert(o, v);
    }
};

}  // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
///@endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_CPP11_MSGPACK_TUPLE_HPP
