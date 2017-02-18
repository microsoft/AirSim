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
#ifndef MSGPACK_ADAPTOR_BASE_HPP
#define MSGPACK_ADAPTOR_BASE_HPP

#include "rpc/msgpack/object_fwd.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

template <typename Stream>
class packer;

namespace adaptor {

// Adaptor functors

template <typename T, typename Enabler = void>
struct convert {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, T& v) const;
};

template <typename T, typename Enabler = void>
struct pack {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, T const& v) const;
};

template <typename T, typename Enabler = void>
struct object {
    void operator()(clmdep_msgpack::object& o, T const& v) const;
};

template <typename T, typename Enabler = void>
struct object_with_zone {
    void operator()(clmdep_msgpack::object::with_zone& o, T const& v) const;
};

} // namespace adaptor

// operators

template <typename T>
inline
clmdep_msgpack::object const& operator>> (clmdep_msgpack::object const& o, T& v) {
    return adaptor::convert<T>()(o, v);
}

template <typename Stream, typename T>
inline
clmdep_msgpack::packer<Stream>& operator<< (clmdep_msgpack::packer<Stream>& o, T const& v) {
    return adaptor::pack<T>()(o, v);
}

template <typename T>
inline
void operator<< (clmdep_msgpack::object& o, T const& v) {
    adaptor::object<T>()(o, v);
}

template <typename T>
inline
void operator<< (clmdep_msgpack::object::with_zone& o, T const& v) {
    adaptor::object_with_zone<T>()(o, v);
}

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack


#endif // MSGPACK_ADAPTOR_BASE_HPP
