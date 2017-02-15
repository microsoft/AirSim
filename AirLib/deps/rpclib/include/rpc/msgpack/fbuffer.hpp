//
// MessagePack for C++ FILE* buffer adaptor
//
// Copyright (C) 2013 Vladimir Volodko
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
#ifndef MSGPACK_FBUFFER_HPP__
#define MSGPACK_FBUFFER_HPP__

#include "rpc/msgpack/versioning.hpp"

#include <cstdio>
#include <stdexcept>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

class fbuffer {
public:
    explicit fbuffer(FILE* file) : m_file(file) { }

public:
    void write(const char* buf, unsigned int len)
    {
        if (1 != fwrite(buf, len, 1, m_file)) {
            throw std::runtime_error("fwrite() failed");
        }
    }

    FILE* file() const
    {
        return m_file;
    }

#if defined(MSGPACK_USE_CPP03)
private:
    fbuffer(const fbuffer&);
    fbuffer& operator=(const fbuffer&);
#else  // defined(MSGPACK_USE_CPP03)
    fbuffer(const fbuffer&) = delete;
    fbuffer& operator=(const fbuffer&) = delete;
#endif // defined(MSGPACK_USE_CPP03)

private:
    FILE* m_file;
};

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif /* msgpack/fbuffer.hpp */
