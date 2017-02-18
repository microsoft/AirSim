/*
 * MessagePack for C FILE* buffer adaptor
 *
 * Copyright (C) 2013 Vladimir Volodko
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
#ifndef MSGPACK_FBUFFER_H
#define MSGPACK_FBUFFER_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup msgpack_fbuffer FILE* buffer
 * @ingroup msgpack_buffer
 * @{
 */

static inline int msgpack_fbuffer_write(void* data, const char* buf, unsigned int len)
{
    return (1 == fwrite(buf, len, 1, (FILE *)data)) ? 0 : -1;
}

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* msgpack/fbuffer.h */
