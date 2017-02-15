#pragma once

#ifndef BOOL_H_QLG6S5XZ
#define BOOL_H_QLG6S5XZ

#include "rpc/detail/constant.h"

namespace rpc {
namespace detail {

template<bool B>
using bool_ = constant<bool, B>;

using true_ = bool_<true>;

using false_ = bool_<false>;

}
}

#endif /* end of include guard: BOOL_H_QLG6S5XZ */
