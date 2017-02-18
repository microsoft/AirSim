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
#ifndef MSGPACK_TYPE_FLOAT_HPP
#define MSGPACK_TYPE_FLOAT_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/object_fwd.hpp"
#include <vector>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

// FIXME check overflow, underflow

namespace adaptor {

template <>
struct convert<float> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, float& v) const {
        if(o.type == clmdep_msgpack::type::FLOAT) {
            v = static_cast<float>(o.via.f64);
        }
        else if (o.type == clmdep_msgpack::type::POSITIVE_INTEGER) {
            v = static_cast<float>(o.via.u64);
        }
        else if (o.type == clmdep_msgpack::type::NEGATIVE_INTEGER) {
            v = static_cast<float>(o.via.i64);
        }
        else {
            throw clmdep_msgpack::type_error();
        }
        return o;
    }
};

template <>
struct pack<float> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const float& v) const {
        o.pack_float(v);
        return o;
    }
};


template <>
struct convert<double> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, double& v) const {
        if(o.type == clmdep_msgpack::type::FLOAT) {
            v = o.via.f64;
        }
        else if (o.type == clmdep_msgpack::type::POSITIVE_INTEGER) {
            v = static_cast<double>(o.via.u64);
        }
        else if (o.type == clmdep_msgpack::type::NEGATIVE_INTEGER) {
            v = static_cast<double>(o.via.i64);
        }
        else {
            throw clmdep_msgpack::type_error();
        }
        return o;
    }
};

template <>
struct pack<double> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const double& v) const {
        o.pack_double(v);
        return o;
    }
};


template <>
struct object<float> {
    void operator()(clmdep_msgpack::object& o, float v) const {
        o.type = clmdep_msgpack::type::FLOAT;
        o.via.f64 = static_cast<double>(v);
    }
};

template <>
struct object<double> {
    void operator()(clmdep_msgpack::object& o, double v) const {
        o.type = clmdep_msgpack::type::FLOAT;
        o.via.f64 = v;
    }
};

template <>
struct object_with_zone<float> {
    void operator()(clmdep_msgpack::object::with_zone& o, float v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

template <>
struct object_with_zone<double> {
    void operator()(clmdep_msgpack::object::with_zone& o, double v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_TYPE_FLOAT_HPP
