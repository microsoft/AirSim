//
// MessagePack for C++ deflate buffer implementation
//
// Copyright (C) 2010-2013 FURUHASHI Sadayuki and KONDO Takatoshi
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
#ifndef MSGPACK_ZBUFFER_HPP
#define MSGPACK_ZBUFFER_HPP

#include "rpc/msgpack/versioning.hpp"

#include <stdexcept>
#include <zlib.h>

#ifndef MSGPACK_ZBUFFER_RESERVE_SIZE
#define MSGPACK_ZBUFFER_RESERVE_SIZE 512
#endif

#ifndef MSGPACK_ZBUFFER_INIT_SIZE
#define MSGPACK_ZBUFFER_INIT_SIZE 8192
#endif

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

class zbuffer {
public:
    zbuffer(int level = Z_DEFAULT_COMPRESSION,
            size_t init_size = MSGPACK_ZBUFFER_INIT_SIZE)
        : m_data(nullptr), m_init_size(init_size)
    {
        m_stream.zalloc = Z_NULL;
        m_stream.zfree = Z_NULL;
        m_stream.opaque = Z_NULL;
        m_stream.next_out = Z_NULL;
        m_stream.avail_out = 0;
        if(deflateInit(&m_stream, level) != Z_OK) {
            throw std::bad_alloc();
        }
    }

    ~zbuffer()
    {
        deflateEnd(&m_stream);
        ::free(m_data);
    }

public:
    void write(const char* buf, size_t len)
    {
        m_stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(buf));
        m_stream.avail_in = len;

        while(m_stream.avail_in > 0) {
            if(m_stream.avail_out < MSGPACK_ZBUFFER_RESERVE_SIZE) {
                if(!expand()) {
                    throw std::bad_alloc();
                }
            }

            if(deflate(&m_stream, Z_NO_FLUSH) != Z_OK) {
                throw std::bad_alloc();
            }
        }
    }

    char* flush()
    {
        while(true) {
            switch(deflate(&m_stream, Z_FINISH)) {
            case Z_STREAM_END:
                return m_data;
            case Z_OK:
                if(!expand()) {
                    throw std::bad_alloc();
                }
                break;
            default:
                throw std::bad_alloc();
            }
        }
    }

    char* data()
    {
        return m_data;
    }

    const char* data() const
    {
        return m_data;
    }

    size_t size() const
    {
        return reinterpret_cast<char*>(m_stream.next_out) - m_data;
    }

    void reset()
    {
        if(deflateReset(&m_stream) != Z_OK) {
            throw std::bad_alloc();
        }
        reset_buffer();
    }

    void reset_buffer()
    {
        m_stream.avail_out += reinterpret_cast<char*>(m_stream.next_out) - m_data;
        m_stream.next_out = reinterpret_cast<Bytef*>(m_data);
    }

    char* release_buffer()
    {
        char* tmp = m_data;
        m_data = nullptr;
        m_stream.next_out = nullptr;
        m_stream.avail_out = 0;
        return tmp;
    }

private:
    bool expand()
    {
        size_t used = reinterpret_cast<char*>(m_stream.next_out) - m_data;
        size_t csize = used + m_stream.avail_out;
        size_t nsize = (csize == 0) ? m_init_size : csize * 2;

        char* tmp = static_cast<char*>(::realloc(m_data, nsize));
        if(tmp == nullptr) {
            return false;
        }

        m_data = tmp;
        m_stream.next_out  = reinterpret_cast<Bytef*>(tmp + used);
        m_stream.avail_out = nsize - used;

        return true;
    }
#if defined(MSGPACK_USE_CPP03)
private:
    zbuffer(const zbuffer&);
    zbuffer& operator=(const zbuffer&);
#else  // defined(MSGPACK_USE_CPP03)
    zbuffer(const zbuffer&) = delete;
    zbuffer& operator=(const zbuffer&) = delete;
#endif // defined(MSGPACK_USE_CPP03)

private:
    z_stream m_stream;
    char* m_data;
    size_t m_init_size;
};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif /* msgpack/zbuffer.hpp */
