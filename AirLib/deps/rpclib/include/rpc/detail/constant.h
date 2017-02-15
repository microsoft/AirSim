#pragma once

#ifndef CONSTANT_H_5CXUYJEW
#define CONSTANT_H_5CXUYJEW

#include <type_traits>

namespace rpc {
namespace detail {

template<typename T, T I>
struct constant : std::integral_constant<T, I> {};

}
}

#endif /* end of include guard: CONSTANT_H_5CXUYJEW */
