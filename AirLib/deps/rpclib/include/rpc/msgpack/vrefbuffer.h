/*
 * MessagePack for C zero-copy buffer implementation
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
#ifndef MSGPACK_VREFBUFFER_H
#define MSGPACK_VREFBUFFER_H

#include "zone.h"
#include <stdlib.h>

#ifndef _WIN32
#include <sys/uio.h>
#else
struct iovec {
    void  *iov_base;
    size_t iov_len;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup msgpack_vrefbuffer Vectored Referencing buffer
 * @ingroup msgpack_buffer
 * @{
 */

struct msgpack_vrefbuffer_chunk;
typedef struct msgpack_vrefbuffer_chunk msgpack_vrefbuffer_chunk;

typedef struct msgpack_vrefbuffer_inner_buffer {
    size_t free;
    char*  ptr;
    msgpack_vrefbuffer_chunk* head;
} msgpack_vrefbuffer_inner_buffer;

typedef struct msgpack_vrefbuffer {
    struct iovec* tail;
    struct iovec* end;
    struct iovec* array;

    size_t chunk_size;
    size_t ref_size;

    msgpack_vrefbuffer_inner_buffer inner_buffer;
} msgpack_vrefbuffer;


#ifndef MSGPACK_VREFBUFFER_REF_SIZE
#define MSGPACK_VREFBUFFER_REF_SIZE 32
#endif

#ifndef MSGPACK_VREFBUFFER_CHUNK_SIZE
#define MSGPACK_VREFBUFFER_CHUNK_SIZE 8192
#endif

MSGPACK_DLLEXPORT
bool msgpack_vrefbuffer_init(msgpack_vrefbuffer* vbuf,
        size_t ref_size, size_t chunk_size);
MSGPACK_DLLEXPORT
void msgpack_vrefbuffer_destroy(msgpack_vrefbuffer* vbuf);

static inline msgpack_vrefbuffer* msgpack_vrefbuffer_new(size_t ref_size, size_t chunk_size);
static inline void msgpack_vrefbuffer_free(msgpack_vrefbuffer* vbuf);

static inline int msgpack_vrefbuffer_write(void* data, const char* buf, size_t len);

static inline const struct iovec* msgpack_vrefbuffer_vec(const msgpack_vrefbuffer* vref);
static inline size_t msgpack_vrefbuffer_veclen(const msgpack_vrefbuffer* vref);

MSGPACK_DLLEXPORT
int msgpack_vrefbuffer_append_copy(msgpack_vrefbuffer* vbuf,
        const char* buf, size_t len);

MSGPACK_DLLEXPORT
int msgpack_vrefbuffer_append_ref(msgpack_vrefbuffer* vbuf,
        const char* buf, size_t len);

MSGPACK_DLLEXPORT
int msgpack_vrefbuffer_migrate(msgpack_vrefbuffer* vbuf, msgpack_vrefbuffer* to);

MSGPACK_DLLEXPORT
void msgpack_vrefbuffer_clear(msgpack_vrefbuffer* vref);

/** @} */


static inline msgpack_vrefbuffer* msgpack_vrefbuffer_new(size_t ref_size, size_t chunk_size)
{
    msgpack_vrefbuffer* vbuf = (msgpack_vrefbuffer*)malloc(sizeof(msgpack_vrefbuffer));
    if (vbuf == NULL) return NULL;
    if(!msgpack_vrefbuffer_init(vbuf, ref_size, chunk_size)) {
        free(vbuf);
        return NULL;
    }
    return vbuf;
}

static inline void msgpack_vrefbuffer_free(msgpack_vrefbuffer* vbuf)
{
    if(vbuf == NULL) { return; }
    msgpack_vrefbuffer_destroy(vbuf);
    free(vbuf);
}

static inline int msgpack_vrefbuffer_write(void* data, const char* buf, size_t len)
{
    msgpack_vrefbuffer* vbuf = (msgpack_vrefbuffer*)data;

    if(len < vbuf->ref_size) {
        return msgpack_vrefbuffer_append_copy(vbuf, buf, len);
    } else {
        return msgpack_vrefbuffer_append_ref(vbuf, buf, len);
    }
}

static inline const struct iovec* msgpack_vrefbuffer_vec(const msgpack_vrefbuffer* vref)
{
    return vref->array;
}

static inline size_t msgpack_vrefbuffer_veclen(const msgpack_vrefbuffer* vref)
{
    return (size_t)(vref->tail - vref->array);
}


#ifdef __cplusplus
}
#endif

#endif /* msgpack/vrefbuffer.h */

