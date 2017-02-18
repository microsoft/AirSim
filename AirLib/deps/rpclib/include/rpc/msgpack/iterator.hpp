//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 MIZUKI Hirata
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

#ifndef MSGPACK_ITERATOR_HPP
#define MSGPACK_ITERATOR_HPP
#if !defined(MSGPACK_USE_CPP03)

#include <rpc/msgpack/object_fwd.hpp>

namespace clmdep_msgpack
{
    /// @cond
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)
    {
    /// @endcond
        inline object_kv* begin(object_map &map) { return map.ptr; }
        inline const object_kv* begin(const object_map &map) { return map.ptr; }
        inline object_kv* end(object_map &map) { return map.ptr + map.size; }
        inline const object_kv* end(const object_map &map) { return map.ptr + map.size; }

        inline object* begin(object_array &array) { return array.ptr; }
        inline const object* begin(const object_array &array) { return array.ptr; }
        inline object* end(object_array &array) { return array.ptr + array.size; }
        inline const object* end(const object_array &array) { return array.ptr + array.size; }
    /// @cond
    }
    /// @endcond
}

#endif // !defined(MSGPACK_USE_CPP03)
#endif // MSGPACK_ITERATOR_HPP
