//
// MessagePack for C++ zero-copy buffer implementation
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
#ifndef MSGPACK_VREFBUFFER_HPP
#define MSGPACK_VREFBUFFER_HPP

#include "rpc/msgpack/versioning.hpp"

#include <stdexcept>

#ifndef MSGPACK_VREFBUFFER_REF_SIZE
#define MSGPACK_VREFBUFFER_REF_SIZE 32
#endif

#ifndef MSGPACK_VREFBUFFER_CHUNK_SIZE
#define MSGPACK_VREFBUFFER_CHUNK_SIZE 8192
#endif

#ifndef _WIN32
#include <sys/uio.h>
#else
struct iovec {
    void  *iov_base;
    size_t iov_len;
};
#endif

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace detail {
    // int64, uint64, double
    std::size_t const packer_max_buffer_size = 9;
} // detail

class vrefbuffer {
private:
    struct chunk {
        chunk* next;
    };
    struct inner_buffer {
        size_t free;
        char*  ptr;
        chunk* head;
    };
public:
    vrefbuffer(size_t ref_size = MSGPACK_VREFBUFFER_REF_SIZE,
               size_t chunk_size = MSGPACK_VREFBUFFER_CHUNK_SIZE)
        :m_ref_size(std::max(ref_size, detail::packer_max_buffer_size + 1)),
         m_chunk_size(chunk_size)
    {
        size_t nfirst = (sizeof(iovec) < 72/2) ?
            72 / sizeof(iovec) : 8;

        iovec* array = static_cast<iovec*>(::malloc(
            sizeof(iovec) * nfirst));
        if(!array) {
            throw std::bad_alloc();
        }

        m_tail  = array;
        m_end   = array + nfirst;
        m_array = array;

        chunk* c = static_cast<chunk*>(::malloc(sizeof(chunk) + chunk_size));
        if(!c) {
            ::free(array);
            throw std::bad_alloc();
        }
        inner_buffer* const ib = &m_inner_buffer;

        ib->free = chunk_size;
        ib->ptr      = reinterpret_cast<char*>(c) + sizeof(chunk);
        ib->head = c;
        c->next = nullptr;

    }

    ~vrefbuffer()
    {
        chunk* c = m_inner_buffer.head;
        while(true) {
            chunk* n = c->next;
            ::free(c);
            if(n != NULL) {
                c = n;
            } else {
                break;
            }
        }
        ::free(m_array);
    }

public:
    void write(const char* buf, size_t len)
    {
        if(len < m_ref_size) {
            append_copy(buf, len);
        } else {
            append_ref(buf, len);
        }
    }

    void append_ref(const char* buf, size_t len)
    {
        if(m_tail == m_end) {
            const size_t nused = m_tail - m_array;
            const size_t nnext = nused * 2;

            iovec* nvec = static_cast<iovec*>(::realloc(
                m_array, sizeof(iovec)*nnext));
            if(!nvec) {
                throw std::bad_alloc();
            }

            m_array = nvec;
            m_end   = nvec + nnext;
            m_tail  = nvec + nused;
        }

        m_tail->iov_base = const_cast<char*>(buf);
        m_tail->iov_len     = len;
        ++m_tail;
    }

    void append_copy(const char* buf, size_t len)
    {
        inner_buffer* const ib = &m_inner_buffer;

        if(ib->free < len) {
            size_t sz = m_chunk_size;
            if(sz < len) {
                sz = len;
            }

            chunk* c = static_cast<chunk*>(::malloc(sizeof(chunk) + sz));
            if(!c) {
                throw std::bad_alloc();
            }

            c->next = ib->head;
            ib->head = c;
            ib->free = sz;
            ib->ptr      = reinterpret_cast<char*>(c) + sizeof(chunk);
        }

        char* m = ib->ptr;
        std::memcpy(m, buf, len);
        ib->free -= len;
        ib->ptr      += len;

        if(m_tail != m_array && m ==
            static_cast<const char*>(
                const_cast<const void *>((m_tail - 1)->iov_base)
            ) + (m_tail - 1)->iov_len) {
            (m_tail - 1)->iov_len += len;
            return;
        } else {
            append_ref( m, len);
        }
    }

    const struct iovec* vector() const
    {
        return m_array;
    }

    size_t vector_size() const
    {
        return m_tail - m_array;
    }

    void migrate(vrefbuffer* to)
    {
        size_t sz = m_chunk_size;

        chunk* empty = static_cast<chunk*>(::malloc(sizeof(chunk) + sz));
        if(!empty) {
            throw std::bad_alloc();
        }

        empty->next = nullptr;

        const size_t nused = m_tail - m_array;
        if(to->m_tail + nused < m_end) {
            const size_t tosize = to->m_tail - to->m_array;
            const size_t reqsize = nused + tosize;
            size_t nnext = (to->m_end - to->m_array) * 2;
            while(nnext < reqsize) {
                size_t tmp_nnext = nnext * 2;
                if (tmp_nnext <= nnext) {
                    nnext = reqsize;
                    break;
                }
                nnext = tmp_nnext;
            }

            iovec* nvec = static_cast<iovec*>(::realloc(
                to->m_array, sizeof(iovec)*nnext));
            if(!nvec) {
                ::free(empty);
                throw std::bad_alloc();
            }

            to->m_array = nvec;
            to->m_end   = nvec + nnext;
            to->m_tail  = nvec + tosize;
        }

        std::memcpy(to->m_tail, m_array, sizeof(iovec)*nused);

        to->m_tail += nused;
        m_tail = m_array;


        inner_buffer* const ib = &m_inner_buffer;
        inner_buffer* const toib = &to->m_inner_buffer;

        chunk* last = ib->head;
        while(last->next) {
            last = last->next;
        }
        last->next = toib->head;
        toib->head = ib->head;

        if(toib->free < ib->free) {
            toib->free = ib->free;
            toib->ptr  = ib->ptr;
        }

        ib->head = empty;
        ib->free = sz;
        ib->ptr      = reinterpret_cast<char*>(empty) + sizeof(chunk);

    }

    void clear()
    {
        chunk* c = m_inner_buffer.head->next;
        chunk* n;
        while(c) {
            n = c->next;
            ::free(c);
            c = n;
        }

        inner_buffer* const ib = &m_inner_buffer;
        c = ib->head;
        c->next = nullptr;
        ib->free = m_chunk_size;
        ib->ptr      = reinterpret_cast<char*>(c) + sizeof(chunk);

        m_tail = m_array;
    }

#if defined(MSGPACK_USE_CPP03)
private:
    vrefbuffer(const vrefbuffer&);
    vrefbuffer& operator=(const vrefbuffer&);
#else  // defined(MSGPACK_USE_CPP03)
    vrefbuffer(const vrefbuffer&) = delete;
    vrefbuffer& operator=(const vrefbuffer&) = delete;
#endif // defined(MSGPACK_USE_CPP03)

private:
    iovec* m_tail;
    iovec* m_end;
    iovec* m_array;

    size_t m_ref_size;
    size_t m_chunk_size;

    inner_buffer m_inner_buffer;

};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif /* msgpack/vrefbuffer.hpp */
