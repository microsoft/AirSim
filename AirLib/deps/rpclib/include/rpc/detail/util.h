#pragma once

#ifndef UTIL_H_YRIZ63UJ
#define UTIL_H_YRIZ63UJ

#include "rpc/msgpack.hpp"

namespace rpc {
namespace detail {
template <typename T> RPCLIB_MSGPACK::object_handle pack(T &&o) {
    auto z = std::make_unique<RPCLIB_MSGPACK::zone>();
    RPCLIB_MSGPACK::object obj(std::forward<T>(o), *z);
    return RPCLIB_MSGPACK::object_handle(obj, std::move(z));
}
} /* detail */
} /* rpc  */

#endif /* end of include guard: UTIL_H_YRIZ63UJ */
