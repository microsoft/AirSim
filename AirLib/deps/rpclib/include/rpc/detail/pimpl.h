#pragma once

#ifndef PIMPL_H_TV7E3C9K
#define PIMPL_H_TV7E3C9K

#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>

namespace rpc {
namespace detail {
template <std::size_t Size, int Align> struct aligned_storage {
    typedef typename std::aligned_storage<Size, Align>::type type;
};

template <std::size_t Size> struct aligned_storage<Size, -1> {
    typedef typename std::aligned_storage<Size>::type type;
};

template <typename T, std::size_t ReqSize, std::size_t Size, int ReqAlignment,
          int Alignment>
struct ptr_checker {
    static_assert(ReqSize <= Size, "pimpl_ptr: T has a different size!");
    static_assert(ReqAlignment == Alignment,
                  "pimpl_ptr: T has a different alignment!");
};

template <typename T, std::size_t Size, int Align = -1> class pimpl_ptr {
    typename aligned_storage<Size, Align>::type data;

public:
#define PIMPL_PTR_CHECK_()                                                     \
    ptr_checker<T, sizeof(T), Size, Align, alignof(T)>                 \
        rpc_ptr_checker; (void) rpc_ptr_checker;

    template <typename... Args> pimpl_ptr(Args &&... args) {
        PIMPL_PTR_CHECK_();
        new (&data) T(std::forward<Args>(args)...);
    }
    pimpl_ptr(pimpl_ptr const &o) {
        PIMPL_PTR_CHECK_();
        new (&data) T(*o);
    }
    pimpl_ptr(pimpl_ptr &&o) {
        PIMPL_PTR_CHECK_();
        new (&data) T(std::move(*o));
    }
    ~pimpl_ptr() {
        PIMPL_PTR_CHECK_();
        reinterpret_cast<T *>(&data)->~T();
    }

    pimpl_ptr &operator=(pimpl_ptr const &o) {
        PIMPL_PTR_CHECK_();
        **this = *o;
        return *this;
    }
    pimpl_ptr &operator=(pimpl_ptr &&o) {
        PIMPL_PTR_CHECK_();
        **this = std::move(*o);
        return *this;
    }

    T &operator*() {
        PIMPL_PTR_CHECK_();
        return *reinterpret_cast<T *>(&data);
    }
    T const &operator*() const {
        PIMPL_PTR_CHECK_();
        return *reinterpret_cast<T const *>(&data);
    }
    T *operator->() {
        PIMPL_PTR_CHECK_();
        return &**this;
    }
    T const *operator->() const {
        PIMPL_PTR_CHECK_();
        return &**this;
    }

    friend inline void swap(pimpl_ptr &lhs, pimpl_ptr &rhs) {
        using std::swap;
        PIMPL_PTR_CHECK_();
        swap(*lhs, *rhs);
    }
#undef PIMPL_PTR_CHECK_
};
}
}

#if defined(i386) || defined(__i386) || defined(__i386__) || defined(_M_IX86)
#define RPCLIB_DEF_ALIGN 4
#elif defined(__amd64) || defined(__amd64__) || defined(__x86_64) || defined(__x86_64__) || defined(_M_X64) || defined(_M_AMD64)
#define RPCLIB_DEF_ALIGN 8
#else
#error "Unkown architecture"
#endif

#define RPCLIB_DECL_PIMPL(Size)                                                \
    struct impl;                                                               \
    detail::pimpl_ptr<impl, Size, RPCLIB_DEF_ALIGN> pimpl;

#endif /* end of include guard: PIMPL_H_TV7E3C9K */
