#pragma once

#ifndef DEV_UTILS_H_JQSWE2OS
#define DEV_UTILS_H_JQSWE2OS

#ifdef RPCLIB_LINUX
#include "pthread.h"
#endif

namespace rpc {
namespace detail {
inline void name_thread(std::string const &name) {
    (void)name;
#ifdef RPCLIB_LINUX
    pthread_setname_np(pthread_self(), name.c_str());
#endif
}
} /* detail */
} /* rpc */

#endif /* end of include guard: DEV_UTILS_H_JQSWE2OS */
