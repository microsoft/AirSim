/*
 * MessagePack for C++ version information
 *
 * Copyright (C) 2008-2013 FURUHASHI Sadayuki and Takatoshi Kondo
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#ifndef MSGPACK_VERSION_HPP
#define MSGPACK_VERSION_HPP

#include "version_master.h"

#define MSGPACK_STR(v) #v
#define MSGPACK_VERSION_I(maj, min, rev) MSGPACK_STR(maj) "." MSGPACK_STR(min) "." MSGPACK_STR(rev)

#define MSGPACK_VERSION MSGPACK_VERSION_I(MSGPACK_VERSION_MAJOR, MSGPACK_VERSION_MINOR, MSGPACK_VERSION_REVISION)

inline const char* msgpack_version(void) {
    return MSGPACK_VERSION;
}

inline int msgpack_version_major(void) {
    return MSGPACK_VERSION_MAJOR;
}

inline int msgpack_version_minor(void) {
    return MSGPACK_VERSION_MINOR;
}

inline int msgpack_version_revision(void) {
    return MSGPACK_VERSION_REVISION;
}

#endif /* msgpack/version.hpp */
