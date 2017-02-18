/*
 * MessagePack for C deflate buffer implementation
 *
 * Copyright (C) 2010 FURUHASHI Sadayuki
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
#ifndef MSGPACK_ZBUFFER_H
#define MSGPACK_ZBUFFER_H

#include "sysdep.h"
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup msgpack_zbuffer Compressed buffer
 * @ingroup msgpack_buffer
 * @{
 */

typedef struct msgpack_zbuffer {
    z_stream stream;
    char* data;
    size_t init_size;
} msgpack_zbuffer;

#ifndef MSGPACK_ZBUFFER_INIT_SIZE
#define MSGPACK_ZBUFFER_INIT_SIZE 8192
#endif

static inline bool msgpack_zbuffer_init(
    msgpack_zbuffer* zbuf, int level, size_t init_size);
static inline void msgpack_zbuffer_destroy(msgpack_zbuffer* zbuf);

static inline msgpack_zbuffer* msgpack_zbuffer_new(int level, size_t init_size);
static inline void msgpack_zbuffer_free(msgpack_zbuffer* zbuf);

static inline char* msgpack_zbuffer_flush(msgpack_zbuffer* zbuf);

static inline const char* msgpack_zbuffer_data(const msgpack_zbuffer* zbuf);
static inline size_t msgpack_zbuffer_size(const msgpack_zbuffer* zbuf);

static inline bool msgpack_zbuffer_reset(msgpack_zbuffer* zbuf);
static inline void msgpack_zbuffer_reset_buffer(msgpack_zbuffer* zbuf);
static inline char* msgpack_zbuffer_release_buffer(msgpack_zbuffer* zbuf);


#ifndef MSGPACK_ZBUFFER_RESERVE_SIZE
#define MSGPACK_ZBUFFER_RESERVE_SIZE 512
#endif

static inline int msgpack_zbuffer_write(void* data, const char* buf, size_t len);

static inline bool msgpack_zbuffer_expand(msgpack_zbuffer* zbuf);


static inline bool msgpack_zbuffer_init(msgpack_zbuffer* zbuf,
        int level, size_t init_size)
{
    memset(zbuf, 0, sizeof(msgpack_zbuffer));
    zbuf->init_size = init_size;
    if(deflateInit(&zbuf->stream, level) != Z_OK) {
        free(zbuf->data);
        return false;
    }
    return true;
}

static inline void msgpack_zbuffer_destroy(msgpack_zbuffer* zbuf)
{
    deflateEnd(&zbuf->stream);
    free(zbuf->data);
}

static inline msgpack_zbuffer* msgpack_zbuffer_new(int level, size_t init_size)
{
    msgpack_zbuffer* zbuf = (msgpack_zbuffer*)malloc(sizeof(msgpack_zbuffer));
    if (zbuf == NULL) return NULL;
    if(!msgpack_zbuffer_init(zbuf, level, init_size)) {
        free(zbuf);
        return NULL;
    }
    return zbuf;
}

static inline void msgpack_zbuffer_free(msgpack_zbuffer* zbuf)
{
    if(zbuf == NULL) { return; }
    msgpack_zbuffer_destroy(zbuf);
    free(zbuf);
}

static inline bool msgpack_zbuffer_expand(msgpack_zbuffer* zbuf)
{
    size_t used = (char*)zbuf->stream.next_out - zbuf->data;
    size_t csize = used + zbuf->stream.avail_out;
    size_t nsize = (csize == 0) ? zbuf->init_size : csize * 2;

    char* tmp = (char*)realloc(zbuf->data, nsize);
    if(tmp == NULL) {
        return false;
    }

    zbuf->data = tmp;
    zbuf->stream.next_out  = (Bytef*)(tmp + used);
    zbuf->stream.avail_out = nsize - used;

    return true;
}

static inline int msgpack_zbuffer_write(void* data, const char* buf, size_t len)
{
    msgpack_zbuffer* zbuf = (msgpack_zbuffer*)data;

    zbuf->stream.next_in = (Bytef*)buf;
    zbuf->stream.avail_in = len;

    while(zbuf->stream.avail_in > 0) {
        if(zbuf->stream.avail_out < MSGPACK_ZBUFFER_RESERVE_SIZE) {
            if(!msgpack_zbuffer_expand(zbuf)) {
                return -1;
            }
        }

        if(deflate(&zbuf->stream, Z_NO_FLUSH) != Z_OK) {
            return -1;
        }
    }

    return 0;
}

static inline char* msgpack_zbuffer_flush(msgpack_zbuffer* zbuf)
{
    while(true) {
        switch(deflate(&zbuf->stream, Z_FINISH)) {
        case Z_STREAM_END:
            return zbuf->data;
        case Z_OK:
            if(!msgpack_zbuffer_expand(zbuf)) {
                return NULL;
            }
            break;
        default:
            return NULL;
        }
    }
}

static inline const char* msgpack_zbuffer_data(const msgpack_zbuffer* zbuf)
{
    return zbuf->data;
}

static inline size_t msgpack_zbuffer_size(const msgpack_zbuffer* zbuf)
{
    return (char*)zbuf->stream.next_out - zbuf->data;
}

static inline void msgpack_zbuffer_reset_buffer(msgpack_zbuffer* zbuf)
{
    zbuf->stream.avail_out += (char*)zbuf->stream.next_out - zbuf->data;
    zbuf->stream.next_out = (Bytef*)zbuf->data;
}

static inline bool msgpack_zbuffer_reset(msgpack_zbuffer* zbuf)
{
    if(deflateReset(&zbuf->stream) != Z_OK) {
        return false;
    }
    msgpack_zbuffer_reset_buffer(zbuf);
    return true;
}

static inline char* msgpack_zbuffer_release_buffer(msgpack_zbuffer* zbuf)
{
    char* tmp = zbuf->data;
    zbuf->data = NULL;
    zbuf->stream.next_out = NULL;
    zbuf->stream.avail_out = 0;
    return tmp;
}

/** @} */


#ifdef __cplusplus
}
#endif

#endif /* msgpack/zbuffer.h */

