//
// MessagePack for C++ memory pool
//
// Copyright (C) 2008-2010 FURUHASHI Sadayuki
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
#ifndef MSGPACK_CPP03_ZONE_HPP
#define MSGPACK_CPP03_ZONE_HPP

#include <cstdlib>
#include <memory>
#include <vector>

#include "rpc/msgpack/versioning.hpp"

#ifndef MSGPACK_ZONE_CHUNK_SIZE
#define MSGPACK_ZONE_CHUNK_SIZE 8192
#endif

#ifndef MSGPACK_ZONE_ALIGN
#define MSGPACK_ZONE_ALIGN sizeof(void*)
#endif


namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

class zone {
    struct finalizer {
        finalizer(void (*func)(void*), void* data):m_func(func), m_data(data) {}
        void operator()() { m_func(m_data); }
        void (*m_func)(void*);
        void* m_data;
    };
    struct finalizer_array {
        finalizer_array():m_tail(nullptr), m_end(nullptr), m_array(nullptr) {}
        void call() {
            finalizer* fin = m_tail;
            for(; fin != m_array; --fin) (*(fin-1))();
        }
        ~finalizer_array() {
            call();
            ::free(m_array);
        }
        void clear() {
            call();
            m_tail = m_array;
        }
        void push(void (*func)(void* data), void* data)
        {
            finalizer* fin = m_tail;

            if(fin == m_end) {
                push_expand(func, data);
                return;
            }

            fin->m_func = func;
            fin->m_data = data;

            ++m_tail;
        }
        void push_expand(void (*func)(void*), void* data) {
            const size_t nused = m_end - m_array;
            size_t nnext;
            if(nused == 0) {
                nnext = (sizeof(finalizer) < 72/2) ?
                    72 / sizeof(finalizer) : 8;
            } else {
                nnext = nused * 2;
            }
            finalizer* tmp =
                static_cast<finalizer*>(::realloc(m_array, sizeof(finalizer) * nnext));
            if(!tmp) {
                throw std::bad_alloc();
            }
            m_array     = tmp;
            m_end   = tmp + nnext;
            m_tail  = tmp + nused;
            new (m_tail) finalizer(func, data);

            ++m_tail;
        }
        finalizer* m_tail;
        finalizer* m_end;
        finalizer* m_array;
    };
    struct chunk {
        chunk* m_next;
    };
    struct chunk_list {
        chunk_list(size_t chunk_size)
        {
            chunk* c = static_cast<chunk*>(::malloc(sizeof(chunk) + chunk_size));
            if(!c) {
                throw std::bad_alloc();
            }

            m_head = c;
            m_free = chunk_size;
            m_ptr  = reinterpret_cast<char*>(c) + sizeof(chunk);
            c->m_next = nullptr;
        }
        ~chunk_list()
        {
            chunk* c = m_head;
            while(c) {
                chunk* n = c->m_next;
                ::free(c);
                c = n;
            }
        }
        void clear(size_t chunk_size)
        {
            chunk* c = m_head;
            while(true) {
                chunk* n = c->m_next;
                if(n) {
                    ::free(c);
                    c = n;
                } else {
                    break;
                }
            }
            m_head->m_next = nullptr;
            m_free = chunk_size;
            m_ptr  = reinterpret_cast<char*>(m_head) + sizeof(chunk);
        }
        size_t m_free;
        char* m_ptr;
        chunk* m_head;
    };
    size_t m_chunk_size;
    chunk_list m_chunk_list;
    finalizer_array m_finalizer_array;

public:
    zone(size_t chunk_size = MSGPACK_ZONE_CHUNK_SIZE) /* throw() */;

public:
    void* allocate_align(size_t size, size_t align = MSGPACK_ZONE_ALIGN);
    void* allocate_no_align(size_t size);

    void push_finalizer(void (*func)(void*), void* data);

    template <typename T>
    void push_finalizer(clmdep_msgpack::unique_ptr<T> obj);

    void clear();

    void swap(zone& o);
    static void* operator new(std::size_t size)
    {
        void* p = ::malloc(size);
        if (!p) throw std::bad_alloc();
        return p;
    }
    static void operator delete(void *p) /* throw() */
    {
        ::free(p);
    }
    static void* operator new(std::size_t size, void* place) /* throw() */
    {
        return ::operator new(size, place);
    }
    static void operator delete(void* p, void* place) /* throw() */
    {
        ::operator delete(p, place);
    }
    /// @cond
    
    template <typename T>
    T* allocate();
    
    template <typename T, typename A1>
    T* allocate(A1 a1);
    
    template <typename T, typename A1, typename A2>
    T* allocate(A1 a1, A2 a2);
    
    template <typename T, typename A1, typename A2, typename A3>
    T* allocate(A1 a1, A2 a2, A3 a3);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13, A14 a14);
    
    template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15>
    T* allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13, A14 a14, A15 a15);
    
    /// @endcond

private:
    void undo_allocate(size_t size);

    template <typename T>
    static void object_destruct(void* obj);

    template <typename T>
    static void object_delete(void* obj);

    void* allocate_expand(size_t size);
private:
    zone(const zone&);
    zone& operator=(const zone&);
};

inline zone::zone(size_t chunk_size) /* throw() */ :m_chunk_size(chunk_size), m_chunk_list(m_chunk_size)
{
}

inline void* zone::allocate_align(size_t size, size_t align)
{
    char* aligned =
        reinterpret_cast<char*>(
            reinterpret_cast<size_t>(
                (m_chunk_list.m_ptr + (align - 1))) / align * align);
    size_t adjusted_size = size + (aligned - m_chunk_list.m_ptr);
    if(m_chunk_list.m_free >= adjusted_size) {
        m_chunk_list.m_free -= adjusted_size;
        m_chunk_list.m_ptr  += adjusted_size;
        return aligned;
    }
    return reinterpret_cast<char*>(
        reinterpret_cast<size_t>(
            allocate_expand(size + (align - 1))) / align * align);
}

inline void* zone::allocate_no_align(size_t size)
{
    if(m_chunk_list.m_free < size) {
        return allocate_expand(size);
    }

    char* ptr = m_chunk_list.m_ptr;
    m_chunk_list.m_free -= size;
    m_chunk_list.m_ptr  += size;

    return ptr;
}

inline void* zone::allocate_expand(size_t size)
{
    chunk_list* const cl = &m_chunk_list;

    size_t sz = m_chunk_size;

    while(sz < size) {
        size_t tmp_sz = sz * 2;
        if (tmp_sz <= sz) {
            sz = size;
            break;
        }
        sz = tmp_sz;
    }

    chunk* c = static_cast<chunk*>(::malloc(sizeof(chunk) + sz));
    if (!c) throw std::bad_alloc();

    char* ptr = reinterpret_cast<char*>(c) + sizeof(chunk);

    c->m_next  = cl->m_head;
    cl->m_head = c;
    cl->m_free = sz - size;
    cl->m_ptr  = ptr + size;

    return ptr;
}

inline void zone::push_finalizer(void (*func)(void*), void* data)
{
    m_finalizer_array.push(func, data);
}

template <typename T>
inline void zone::push_finalizer(clmdep_msgpack::unique_ptr<T> obj)
{
    m_finalizer_array.push(&zone::object_delete<T>, obj.release());
}

inline void zone::clear()
{
    m_finalizer_array.clear();
    m_chunk_list.clear(m_chunk_size);
}

inline void zone::swap(zone& o)
{
    using std::swap;
    swap(m_chunk_size, o.m_chunk_size);
    swap(m_chunk_list, o.m_chunk_list);
    swap(m_finalizer_array, o.m_finalizer_array);
}

template <typename T>
void zone::object_destruct(void* obj)
{
    static_cast<T*>(obj)->~T();
}

template <typename T>
void zone::object_delete(void* obj)
{
    delete static_cast<T*>(obj);
}

inline void zone::undo_allocate(size_t size)
{
    m_chunk_list.m_ptr  -= size;
    m_chunk_list.m_free += size;
}

inline std::size_t aligned_size(
    std::size_t size,
    std::size_t align = MSGPACK_ZONE_ALIGN) {
    return (size + align - 1) / align * align;
}

/// @cond

template <typename T>
T* zone::allocate()
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T();
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1>
T* zone::allocate(A1 a1)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2>
T* zone::allocate(A1 a1, A2 a2)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3>
T* zone::allocate(A1 a1, A2 a2, A3 a3)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13, A14 a14)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

template <typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10, typename A11, typename A12, typename A13, typename A14, typename A15>
T* zone::allocate(A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9, A10 a10, A11 a11, A12 a12, A13 a13, A14 a14, A15 a15)
{
    void* x = allocate_align(sizeof(T));
    try {
        m_finalizer_array.push(&zone::object_destruct<T>, x);
    } catch (...) {
        undo_allocate(sizeof(T));
        throw;
    }
    try {
        return new (x) T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15);
    } catch (...) {
        --m_finalizer_array.m_tail;
        undo_allocate(sizeof(T));
        throw;
    }
}

/// @endcond

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_CPP03_ZONE_HPP
