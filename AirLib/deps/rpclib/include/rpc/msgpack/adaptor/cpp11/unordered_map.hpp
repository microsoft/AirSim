//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2014-2015 KONDO Takatoshi
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
#ifndef MSGPACK_TYPE_UNORDERED_MAP_HPP
#define MSGPACK_TYPE_UNORDERED_MAP_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <unordered_map>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct as<
    std::unordered_map<K, V, Hash, Compare, Alloc>,
    typename std::enable_if<clmdep_msgpack::has_as<K>::value && clmdep_msgpack::has_as<V>::value>::type> {
    std::unordered_map<K, V, Hash, Compare, Alloc> operator()(clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        clmdep_msgpack::object_kv* p(o.via.map.ptr);
        clmdep_msgpack::object_kv* const pend(o.via.map.ptr + o.via.map.size);
        std::unordered_map<K, V, Hash, Compare, Alloc> v;
        for (; p != pend; ++p) {
            v.emplace(p->key.as<K>(), p->val.as<V>());
        }
        return v;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct convert<std::unordered_map<K, V, Hash, Compare, Alloc>> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::unordered_map<K, V, Hash, Compare, Alloc>& v) const {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        clmdep_msgpack::object_kv* p(o.via.map.ptr);
        clmdep_msgpack::object_kv* const pend(o.via.map.ptr + o.via.map.size);
        std::unordered_map<K, V, Hash, Compare, Alloc> tmp;
        for(; p != pend; ++p) {
            K key;
            p->key.convert(key);
            p->val.convert(tmp[std::move(key)]);
        }
        v = std::move(tmp);
        return o;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct pack<std::unordered_map<K, V, Hash, Compare, Alloc>> {
    template <typename Stream>
        clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::unordered_map<K, V, Hash, Compare, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_map(size);
        for(typename std::unordered_map<K, V, Hash, Compare, Alloc>::const_iterator it(v.begin()), it_end(v.end());
            it != it_end; ++it) {
            o.pack(it->first);
            o.pack(it->second);
        }
        return o;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct object_with_zone<std::unordered_map<K, V, Hash, Compare, Alloc>> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::unordered_map<K, V, Hash, Compare, Alloc>& v) const {
        o.type = clmdep_msgpack::type::MAP;
        if(v.empty()) {
            o.via.map.ptr  = nullptr;
            o.via.map.size = 0;
        } else {
            uint32_t size = checked_get_container_size(v.size());
            clmdep_msgpack::object_kv* p = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv)*size));
            clmdep_msgpack::object_kv* const pend = p + size;
            o.via.map.ptr  = p;
            o.via.map.size = size;
            typename std::unordered_map<K, V, Hash, Compare, Alloc>::const_iterator it(v.begin());
            do {
                p->key = clmdep_msgpack::object(it->first, o.zone);
                p->val = clmdep_msgpack::object(it->second, o.zone);
                ++p;
                ++it;
            } while(p < pend);
        }
    }
};


template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct as<
    std::unordered_multimap<K, V, Hash, Compare, Alloc>,
    typename std::enable_if<clmdep_msgpack::has_as<K>::value && clmdep_msgpack::has_as<V>::value>::type> {
    std::unordered_multimap<K, V, Hash, Compare, Alloc> operator()(clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        clmdep_msgpack::object_kv* p(o.via.map.ptr);
        clmdep_msgpack::object_kv* const pend(o.via.map.ptr + o.via.map.size);
        std::unordered_multimap<K, V, Hash, Compare, Alloc> v;
        for (; p != pend; ++p) {
            v.emplace(p->key.as<K>(), p->val.as<V>());
        }
        return v;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct convert<std::unordered_multimap<K, V, Hash, Compare, Alloc>> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::unordered_multimap<K, V, Hash, Compare, Alloc>& v) const {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        clmdep_msgpack::object_kv* p(o.via.map.ptr);
        clmdep_msgpack::object_kv* const pend(o.via.map.ptr + o.via.map.size);
        std::unordered_multimap<K, V, Hash, Compare, Alloc> tmp;
        for(; p != pend; ++p) {
            std::pair<K, V> value;
            p->key.convert(value.first);
            p->val.convert(value.second);
            tmp.insert(std::move(value));
        }
        v = std::move(tmp);
        return o;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct pack<std::unordered_multimap<K, V, Hash, Compare, Alloc>> {
    template <typename Stream>
        clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::unordered_multimap<K, V, Hash, Compare, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_map(size);
        for(typename std::unordered_multimap<K, V, Hash, Compare, Alloc>::const_iterator it(v.begin()), it_end(v.end());
            it != it_end; ++it) {
            o.pack(it->first);
            o.pack(it->second);
        }
        return o;
    }
};

template <typename K, typename V, typename Hash, typename Compare, typename Alloc>
struct object_with_zone<std::unordered_multimap<K, V, Hash, Compare, Alloc>> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::unordered_multimap<K, V, Hash, Compare, Alloc>& v) const {
        o.type = clmdep_msgpack::type::MAP;
        if(v.empty()) {
            o.via.map.ptr  = nullptr;
            o.via.map.size = 0;
        } else {
            uint32_t size = checked_get_container_size(v.size());
            clmdep_msgpack::object_kv* p = static_cast<clmdep_msgpack::object_kv*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object_kv)*size));
            clmdep_msgpack::object_kv* const pend = p + size;
            o.via.map.ptr  = p;
            o.via.map.size = size;
            typename std::unordered_multimap<K, V, Hash, Compare, Alloc>::const_iterator it(v.begin());
            do {
                p->key = clmdep_msgpack::object(it->first, o.zone);
                p->val = clmdep_msgpack::object(it->second, o.zone);
                ++p;
                ++it;
            } while(p < pend);
        }
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack


#endif // MSGPACK_TYPE_UNORDERED_MAP_HPP
