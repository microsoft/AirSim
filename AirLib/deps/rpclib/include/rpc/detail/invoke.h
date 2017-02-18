#pragma once

#ifndef INVOKE_H_0CWMPLUE
#define INVOKE_H_0CWMPLUE

namespace rpc {
namespace detail {

template<typename T>
using invoke = typename T::type;

}
}


#endif /* end of include guard: INVOKE_H_0CWMPLUE */
