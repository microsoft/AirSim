//
// MessagePack for C++ simple buffer implementation
//
// Copyright (C) 2008-2013 FURUHASHI Sadayuki and KONDO Takatoshi
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
#ifndef MSGPACK_SBUFFER_HPP
#define MSGPACK_SBUFFER_HPP

#include "rpc/msgpack/versioning.hpp"

#include <stdexcept>

#ifndef MSGPACK_SBUFFER_INIT_SIZE
#define MSGPACK_SBUFFER_INIT_SIZE 8192
#endif

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

class sbuffer {
public:
    sbuffer(size_t initsz = MSGPACK_SBUFFER_INIT_SIZE):m_size(0), m_alloc(initsz)
    {
        if(initsz == 0) {
            m_data = nullptr;
        } else {
            m_data = (char*)::malloc(initsz);
            if(!m_data) {
                throw std::bad_alloc();
            }
        }
    }

    ~sbuffer()
    {
        ::free(m_data);
    }

#if !defined(MSGPACK_USE_CPP03)
    sbuffer(const sbuffer&) = delete;
    sbuffer& operator=(const sbuffer&) = delete;

    sbuffer(sbuffer&& other) :
        m_size(other.m_size), m_data(other.m_data), m_alloc(other.m_alloc)
    {
        other.m_size = other.m_alloc = 0;
        other.m_data = nullptr;
    }

    sbuffer& operator=(sbuffer&& other)
    {
        ::free(m_data);

        m_size = other.m_size;
        m_alloc = other.m_alloc;
        m_data = other.m_data;

        other.m_size = other.m_alloc = 0;
        other.m_data = nullptr;

        return *this;
    }
#endif // !defined(MSGPACK_USE_CPP03)

    void write(const char* buf, size_t len)
    {
        if(m_alloc - m_size < len) {
            expand_buffer(len);
        }
        std::memcpy(m_data + m_size, buf, len);
        m_size += len;
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
        return m_size;
    }

    char* release()
    {
        char* tmp = m_data;
        m_size = 0;
        m_data = nullptr;
        m_alloc = 0;
        return tmp;
    }

    void clear()
    {
        m_size = 0;
    }

private:
    void expand_buffer(size_t len)
    {
        size_t nsize = (m_alloc > 0) ?
                m_alloc * 2 : MSGPACK_SBUFFER_INIT_SIZE;

        while(nsize < m_size + len) {
            size_t tmp_nsize = nsize * 2;
            if (tmp_nsize <= nsize) {
                nsize = m_size + len;
                break;
            }
            nsize = tmp_nsize;
        }

        void* tmp = ::realloc(m_data, nsize);
        if(!tmp) {
            throw std::bad_alloc();
        }

        m_data = static_cast<char*>(tmp);
        m_alloc = nsize;
    }

#if defined(MSGPACK_USE_CPP03)
private:
    sbuffer(const sbuffer&);
    sbuffer& operator=(const sbuffer&);
#endif  // defined(MSGPACK_USE_CPP03)

private:
    size_t m_size;
    char* m_data;
    size_t m_alloc;
};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif /* msgpack/sbuffer.hpp */
