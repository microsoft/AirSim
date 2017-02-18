/*
 * MessagePack for C simple buffer implementation
 *
 * Copyright (C) 2008-2009 FURUHASHI Sadayuki
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
#ifndef MSGPACK_SBUFFER_H
#define MSGPACK_SBUFFER_H

#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup msgpack_sbuffer Simple buffer
 * @ingroup msgpack_buffer
 * @{
 */

typedef struct msgpack_sbuffer {
    size_t size;
    char* data;
    size_t alloc;
} msgpack_sbuffer;

static inline void msgpack_sbuffer_init(msgpack_sbuffer* sbuf)
{
    memset(sbuf, 0, sizeof(msgpack_sbuffer));
}

static inline void msgpack_sbuffer_destroy(msgpack_sbuffer* sbuf)
{
    free(sbuf->data);
}

static inline msgpack_sbuffer* msgpack_sbuffer_new(void)
{
    return (msgpack_sbuffer*)calloc(1, sizeof(msgpack_sbuffer));
}

static inline void msgpack_sbuffer_free(msgpack_sbuffer* sbuf)
{
    if(sbuf == NULL) { return; }
    msgpack_sbuffer_destroy(sbuf);
    free(sbuf);
}

#ifndef MSGPACK_SBUFFER_INIT_SIZE
#define MSGPACK_SBUFFER_INIT_SIZE 8192
#endif

static inline int msgpack_sbuffer_write(void* data, const char* buf, size_t len)
{
    msgpack_sbuffer* sbuf = (msgpack_sbuffer*)data;

    if(sbuf->alloc - sbuf->size < len) {
        void* tmp;
        size_t nsize = (sbuf->alloc) ?
                sbuf->alloc * 2 : MSGPACK_SBUFFER_INIT_SIZE;

        while(nsize < sbuf->size + len) {
            size_t tmp_nsize = nsize * 2;
            if (tmp_nsize <= nsize) {
                nsize = sbuf->size + len;
                break;
            }
            nsize = tmp_nsize;
        }

        tmp = realloc(sbuf->data, nsize);
        if(!tmp) { return -1; }

        sbuf->data = (char*)tmp;
        sbuf->alloc = nsize;
    }

    memcpy(sbuf->data + sbuf->size, buf, len);
    sbuf->size += len;
    return 0;
}

static inline char* msgpack_sbuffer_release(msgpack_sbuffer* sbuf)
{
    char* tmp = sbuf->data;
    sbuf->size = 0;
    sbuf->data = NULL;
    sbuf->alloc = 0;
    return tmp;
}

static inline void msgpack_sbuffer_clear(msgpack_sbuffer* sbuf)
{
    sbuf->size = 0;
}

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* msgpack/sbuffer.h */
