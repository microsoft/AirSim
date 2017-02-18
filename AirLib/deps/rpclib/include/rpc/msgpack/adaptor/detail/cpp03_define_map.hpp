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
#ifndef MSGPACK_CPP03_DEFINE_MAP_HPP
#define MSGPACK_CPP03_DEFINE_MAP_HPP

// BOOST_PP_VARIADICS is defined in boost/preprocessor/config/config.hpp
// http://www.boost.org/libs/preprocessor/doc/ref/variadics.html
// However, supporting compiler detection is not complete. msgpack-c requires
// variadic macro arguments support. So BOOST_PP_VARIADICS is defined here explicitly.
#if !defined(MSGPACK_PP_VARIADICS)
#define MSGPACK_PP_VARIADICS
#endif

#include <rpc/msgpack/preprocessor.hpp>

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/msgpack_tuple.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/object_fwd.hpp"

#define MSGPACK_DEFINE_MAP_EACH_PROC(r, data, elem) \
    MSGPACK_PP_IF( \
        MSGPACK_PP_IS_BEGIN_PARENS(elem), \
        elem, \
        (MSGPACK_PP_STRINGIZE(elem))(elem) \
    )

#define MSGPACK_DEFINE_MAP_IMPL(...) \
    MSGPACK_PP_SEQ_TO_TUPLE( \
        MSGPACK_PP_SEQ_FOR_EACH( \
            MSGPACK_DEFINE_MAP_EACH_PROC, \
            0, \
            MSGPACK_PP_VARIADIC_TO_SEQ(__VA_ARGS__) \
        ) \
    )

#define MSGPACK_DEFINE_MAP(...) \
    template <typename Packer> \
    void msgpack_pack(Packer& pk) const \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_pack(pk); \
    } \
    void msgpack_unpack(clmdep_msgpack::object const& o) \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_unpack(o); \
    }\
    template <typename MSGPACK_OBJECT> \
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& z) const \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_object(o, z); \
    }

#define MSGPACK_BASE_MAP(base) \
    (MSGPACK_PP_STRINGIZE(base))(*const_cast<base *>(static_cast<base const*>(this)))

namespace clmdep_msgpack {
/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond
namespace type {

/// @cond

template <typename A0 = void, typename A1 = void, typename A2 = void, typename A3 = void, typename A4 = void, typename A5 = void, typename A6 = void, typename A7 = void, typename A8 = void, typename A9 = void, typename A10 = void, typename A11 = void, typename A12 = void, typename A13 = void, typename A14 = void, typename A15 = void, typename A16 = void, typename A17 = void, typename A18 = void, typename A19 = void, typename A20 = void, typename A21 = void, typename A22 = void, typename A23 = void, typename A24 = void, typename A25 = void, typename A26 = void, typename A27 = void, typename A28 = void, typename A29 = void, typename A30 = void, typename A31 = void, typename A32 = void>
struct define_map;
/// @endcond

template <>
struct define_map<> {
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(0);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone&) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = nullptr;
        o->via.map.size = 0;
    }
};

/// @cond

template <typename A0, typename A1>
struct define_map<A0, A1> {
    define_map(A0& _a0, A1& _a1) :
        a0(_a0), a1(_a1) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(1);
        
        pk.pack(a0);
        pk.pack(a1);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*1));
        o->via.map.size = 1;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
    }
    
    A0& a0;
    A1& a1;
};

template <typename A0, typename A1, typename A2, typename A3>
struct define_map<A0, A1, A2, A3> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(2);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*2));
        o->via.map.size = 2;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5>
struct define_map<A0, A1, A2, A3, A4, A5> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(3);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*3));
        o->via.map.size = 3;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(4);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*4));
        o->via.map.size = 4;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(5);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*5));
        o->via.map.size = 5;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(6);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*6));
        o->via.map.size = 6;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(7);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*7));
        o->via.map.size = 7;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(8);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*8));
        o->via.map.size = 8;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(9);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*9));
        o->via.map.size = 9;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(10);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*10));
        o->via.map.size = 10;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(11);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*11));
        o->via.map.size = 11;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21, A22& _a22, A23& _a23) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21), a22(_a22), a23(_a23) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(12);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
        pk.pack(a22);
        pk.pack(a23);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a22);
            if (it != kvmap.end()) {
                it->second->convert(a23);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*12));
        o->via.map.size = 12;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
        o->via.map.ptr[11].key = clmdep_msgpack::object(a22, z);
        o->via.map.ptr[11].val = clmdep_msgpack::object(a23, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
    A22& a22;
    A23& a23;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21, A22& _a22, A23& _a23, A24& _a24, A25& _a25) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21), a22(_a22), a23(_a23), a24(_a24), a25(_a25) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(13);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
        pk.pack(a22);
        pk.pack(a23);
        pk.pack(a24);
        pk.pack(a25);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a22);
            if (it != kvmap.end()) {
                it->second->convert(a23);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a24);
            if (it != kvmap.end()) {
                it->second->convert(a25);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*13));
        o->via.map.size = 13;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
        o->via.map.ptr[11].key = clmdep_msgpack::object(a22, z);
        o->via.map.ptr[11].val = clmdep_msgpack::object(a23, z);
        
        o->via.map.ptr[12].key = clmdep_msgpack::object(a24, z);
        o->via.map.ptr[12].val = clmdep_msgpack::object(a25, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
    A22& a22;
    A23& a23;
    A24& a24;
    A25& a25;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21, A22& _a22, A23& _a23, A24& _a24, A25& _a25, A26& _a26, A27& _a27) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21), a22(_a22), a23(_a23), a24(_a24), a25(_a25), a26(_a26), a27(_a27) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(14);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
        pk.pack(a22);
        pk.pack(a23);
        pk.pack(a24);
        pk.pack(a25);
        pk.pack(a26);
        pk.pack(a27);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a22);
            if (it != kvmap.end()) {
                it->second->convert(a23);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a24);
            if (it != kvmap.end()) {
                it->second->convert(a25);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a26);
            if (it != kvmap.end()) {
                it->second->convert(a27);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*14));
        o->via.map.size = 14;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
        o->via.map.ptr[11].key = clmdep_msgpack::object(a22, z);
        o->via.map.ptr[11].val = clmdep_msgpack::object(a23, z);
        
        o->via.map.ptr[12].key = clmdep_msgpack::object(a24, z);
        o->via.map.ptr[12].val = clmdep_msgpack::object(a25, z);
        
        o->via.map.ptr[13].key = clmdep_msgpack::object(a26, z);
        o->via.map.ptr[13].val = clmdep_msgpack::object(a27, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
    A22& a22;
    A23& a23;
    A24& a24;
    A25& a25;
    A26& a26;
    A27& a27;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28, typename A29>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21, A22& _a22, A23& _a23, A24& _a24, A25& _a25, A26& _a26, A27& _a27, A28& _a28, A29& _a29) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21), a22(_a22), a23(_a23), a24(_a24), a25(_a25), a26(_a26), a27(_a27), a28(_a28), a29(_a29) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(15);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
        pk.pack(a22);
        pk.pack(a23);
        pk.pack(a24);
        pk.pack(a25);
        pk.pack(a26);
        pk.pack(a27);
        pk.pack(a28);
        pk.pack(a29);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a22);
            if (it != kvmap.end()) {
                it->second->convert(a23);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a24);
            if (it != kvmap.end()) {
                it->second->convert(a25);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a26);
            if (it != kvmap.end()) {
                it->second->convert(a27);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a28);
            if (it != kvmap.end()) {
                it->second->convert(a29);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*15));
        o->via.map.size = 15;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
        o->via.map.ptr[11].key = clmdep_msgpack::object(a22, z);
        o->via.map.ptr[11].val = clmdep_msgpack::object(a23, z);
        
        o->via.map.ptr[12].key = clmdep_msgpack::object(a24, z);
        o->via.map.ptr[12].val = clmdep_msgpack::object(a25, z);
        
        o->via.map.ptr[13].key = clmdep_msgpack::object(a26, z);
        o->via.map.ptr[13].val = clmdep_msgpack::object(a27, z);
        
        o->via.map.ptr[14].key = clmdep_msgpack::object(a28, z);
        o->via.map.ptr[14].val = clmdep_msgpack::object(a29, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
    A22& a22;
    A23& a23;
    A24& a24;
    A25& a25;
    A26& a26;
    A27& a27;
    A28& a28;
    A29& a29;
};

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28, typename A29, typename A30, typename A31>
struct define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30, A31> {
    define_map(A0& _a0, A1& _a1, A2& _a2, A3& _a3, A4& _a4, A5& _a5, A6& _a6, A7& _a7, A8& _a8, A9& _a9, A10& _a10, A11& _a11, A12& _a12, A13& _a13, A14& _a14, A15& _a15, A16& _a16, A17& _a17, A18& _a18, A19& _a19, A20& _a20, A21& _a21, A22& _a22, A23& _a23, A24& _a24, A25& _a25, A26& _a26, A27& _a27, A28& _a28, A29& _a29, A30& _a30, A31& _a31) :
        a0(_a0), a1(_a1), a2(_a2), a3(_a3), a4(_a4), a5(_a5), a6(_a6), a7(_a7), a8(_a8), a9(_a9), a10(_a10), a11(_a11), a12(_a12), a13(_a13), a14(_a14), a15(_a15), a16(_a16), a17(_a17), a18(_a18), a19(_a19), a20(_a20), a21(_a21), a22(_a22), a23(_a23), a24(_a24), a25(_a25), a26(_a26), a27(_a27), a28(_a28), a29(_a29), a30(_a30), a31(_a31) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        pk.pack_map(16);
        
        pk.pack(a0);
        pk.pack(a1);
        pk.pack(a2);
        pk.pack(a3);
        pk.pack(a4);
        pk.pack(a5);
        pk.pack(a6);
        pk.pack(a7);
        pk.pack(a8);
        pk.pack(a9);
        pk.pack(a10);
        pk.pack(a11);
        pk.pack(a12);
        pk.pack(a13);
        pk.pack(a14);
        pk.pack(a15);
        pk.pack(a16);
        pk.pack(a17);
        pk.pack(a18);
        pk.pack(a19);
        pk.pack(a20);
        pk.pack(a21);
        pk.pack(a22);
        pk.pack(a23);
        pk.pack(a24);
        pk.pack(a25);
        pk.pack(a26);
        pk.pack(a27);
        pk.pack(a28);
        pk.pack(a29);
        pk.pack(a30);
        pk.pack(a31);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.insert(
                std::map<std::string, clmdep_msgpack::object const*>::value_type(
                    std::string(
                        o.via.map.ptr[i].key.via.str.ptr,
                        o.via.map.ptr[i].key.via.str.size),
                    &o.via.map.ptr[i].val
                )
            );
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a0);
            if (it != kvmap.end()) {
                it->second->convert(a1);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a2);
            if (it != kvmap.end()) {
                it->second->convert(a3);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a4);
            if (it != kvmap.end()) {
                it->second->convert(a5);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a6);
            if (it != kvmap.end()) {
                it->second->convert(a7);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a8);
            if (it != kvmap.end()) {
                it->second->convert(a9);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a10);
            if (it != kvmap.end()) {
                it->second->convert(a11);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a12);
            if (it != kvmap.end()) {
                it->second->convert(a13);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a14);
            if (it != kvmap.end()) {
                it->second->convert(a15);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a16);
            if (it != kvmap.end()) {
                it->second->convert(a17);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a18);
            if (it != kvmap.end()) {
                it->second->convert(a19);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a20);
            if (it != kvmap.end()) {
                it->second->convert(a21);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a22);
            if (it != kvmap.end()) {
                it->second->convert(a23);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a24);
            if (it != kvmap.end()) {
                it->second->convert(a25);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a26);
            if (it != kvmap.end()) {
                it->second->convert(a27);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a28);
            if (it != kvmap.end()) {
                it->second->convert(a29);
            }
        }
        
        {
            std::map<std::string, clmdep_msgpack::object const*>::const_iterator it = kvmap.find(a30);
            if (it != kvmap.end()) {
                it->second->convert(a31);
            }
        }
        
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*16));
        o->via.map.size = 16;
        
        o->via.map.ptr[0].key = clmdep_msgpack::object(a0, z);
        o->via.map.ptr[0].val = clmdep_msgpack::object(a1, z);
        
        o->via.map.ptr[1].key = clmdep_msgpack::object(a2, z);
        o->via.map.ptr[1].val = clmdep_msgpack::object(a3, z);
        
        o->via.map.ptr[2].key = clmdep_msgpack::object(a4, z);
        o->via.map.ptr[2].val = clmdep_msgpack::object(a5, z);
        
        o->via.map.ptr[3].key = clmdep_msgpack::object(a6, z);
        o->via.map.ptr[3].val = clmdep_msgpack::object(a7, z);
        
        o->via.map.ptr[4].key = clmdep_msgpack::object(a8, z);
        o->via.map.ptr[4].val = clmdep_msgpack::object(a9, z);
        
        o->via.map.ptr[5].key = clmdep_msgpack::object(a10, z);
        o->via.map.ptr[5].val = clmdep_msgpack::object(a11, z);
        
        o->via.map.ptr[6].key = clmdep_msgpack::object(a12, z);
        o->via.map.ptr[6].val = clmdep_msgpack::object(a13, z);
        
        o->via.map.ptr[7].key = clmdep_msgpack::object(a14, z);
        o->via.map.ptr[7].val = clmdep_msgpack::object(a15, z);
        
        o->via.map.ptr[8].key = clmdep_msgpack::object(a16, z);
        o->via.map.ptr[8].val = clmdep_msgpack::object(a17, z);
        
        o->via.map.ptr[9].key = clmdep_msgpack::object(a18, z);
        o->via.map.ptr[9].val = clmdep_msgpack::object(a19, z);
        
        o->via.map.ptr[10].key = clmdep_msgpack::object(a20, z);
        o->via.map.ptr[10].val = clmdep_msgpack::object(a21, z);
        
        o->via.map.ptr[11].key = clmdep_msgpack::object(a22, z);
        o->via.map.ptr[11].val = clmdep_msgpack::object(a23, z);
        
        o->via.map.ptr[12].key = clmdep_msgpack::object(a24, z);
        o->via.map.ptr[12].val = clmdep_msgpack::object(a25, z);
        
        o->via.map.ptr[13].key = clmdep_msgpack::object(a26, z);
        o->via.map.ptr[13].val = clmdep_msgpack::object(a27, z);
        
        o->via.map.ptr[14].key = clmdep_msgpack::object(a28, z);
        o->via.map.ptr[14].val = clmdep_msgpack::object(a29, z);
        
        o->via.map.ptr[15].key = clmdep_msgpack::object(a30, z);
        o->via.map.ptr[15].val = clmdep_msgpack::object(a31, z);
        
    }
    
    A0& a0;
    A1& a1;
    A2& a2;
    A3& a3;
    A4& a4;
    A5& a5;
    A6& a6;
    A7& a7;
    A8& a8;
    A9& a9;
    A10& a10;
    A11& a11;
    A12& a12;
    A13& a13;
    A14& a14;
    A15& a15;
    A16& a16;
    A17& a17;
    A18& a18;
    A19& a19;
    A20& a20;
    A21& a21;
    A22& a22;
    A23& a23;
    A24& a24;
    A25& a25;
    A26& a26;
    A27& a27;
    A28& a28;
    A29& a29;
    A30& a30;
    A31& a31;
};

/// @endcond

inline define_map<> make_define_map()
{
    return define_map<>();
}

/// @cond

template <typename A0>
inline define_map<A0> make_define_map(A0& a0)
{
    return define_map<A0>(a0);
}

template <typename A0, typename A1>
inline define_map<A0, A1> make_define_map(A0& a0, A1& a1)
{
    return define_map<A0, A1>(a0, a1);
}

template <typename A0, typename A1, typename A2>
inline define_map<A0, A1, A2> make_define_map(A0& a0, A1& a1, A2& a2)
{
    return define_map<A0, A1, A2>(a0, a1, a2);
}

template <typename A0, typename A1, typename A2, typename A3>
inline define_map<A0, A1, A2, A3> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3)
{
    return define_map<A0, A1, A2, A3>(a0, a1, a2, a3);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4>
inline define_map<A0, A1, A2, A3, A4> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4)
{
    return define_map<A0, A1, A2, A3, A4>(a0, a1, a2, a3, a4);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5>
inline define_map<A0, A1, A2, A3, A4, A5> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5)
{
    return define_map<A0, A1, A2, A3, A4, A5>(a0, a1, a2, a3, a4, a5);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6>
inline define_map<A0, A1, A2, A3, A4, A5, A6> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6>(a0, a1, a2, a3, a4, a5, a6);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7>(a0, a1, a2, a3, a4, a5, a6, a7);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8>(a0, a1, a2, a3, a4, a5, a6, a7, a8);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26, A27& a27)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26, A27& a27, A28& a28)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28, typename A29>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26, A27& a27, A28& a28, A29& a29)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28, typename A29, typename A30>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26, A27& a27, A28& a28, A29& a29, A30& a30)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30);
}

template <typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15, typename A16, typename A17, typename A18, typename A19, typename A20, typename A21, typename A22, typename A23, typename A24, typename A25, typename A26, typename A27, typename A28, typename A29, typename A30, typename A31>
inline define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30, A31> make_define_map(A0& a0, A1& a1, A2& a2, A3& a3, A4& a4, A5& a5, A6& a6, A7& a7, A8& a8, A9& a9, A10& a10, A11& a11, A12& a12, A13& a13, A14& a14, A15& a15, A16& a16, A17& a17, A18& a18, A19& a19, A20& a20, A21& a21, A22& a22, A23& a23, A24& a24, A25& a25, A26& a26, A27& a27, A28& a28, A29& a29, A30& a30, A31& a31)
{
    return define_map<A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21, A22, A23, A24, A25, A26, A27, A28, A29, A30, A31>(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31);
}

/// @endcond

}  // namespace type
/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond
}  // namespace clmdep_msgpack

#endif // MSGPACK_CPP03_DEFINE_MAP_HPP
