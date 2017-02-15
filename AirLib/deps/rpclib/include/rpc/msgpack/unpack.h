/*
 * MessagePack for C unpacking routine
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
#ifndef MSGPACK_UNPACKER_H
#define MSGPACK_UNPACKER_H

#include "zone.h"
#include "object.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup msgpack_unpack Deserializer
 * @ingroup msgpack
 * @{
 */

typedef struct msgpack_unpacked {
    msgpack_zone* zone;
    msgpack_object data;
} msgpack_unpacked;

typedef enum {
    MSGPACK_UNPACK_SUCCESS              =  2,
    MSGPACK_UNPACK_EXTRA_BYTES          =  1,
    MSGPACK_UNPACK_CONTINUE             =  0,
    MSGPACK_UNPACK_PARSE_ERROR          = -1,
    MSGPACK_UNPACK_NOMEM_ERROR          = -2
} msgpack_unpack_return;


MSGPACK_DLLEXPORT
msgpack_unpack_return
msgpack_unpack_next(msgpack_unpacked* result,
        const char* data, size_t len, size_t* off);

/** @} */


/**
 * @defgroup msgpack_unpacker Streaming deserializer
 * @ingroup msgpack
 * @{
 */

typedef struct msgpack_unpacker {
    char* buffer;
    size_t used;
    size_t free;
    size_t off;
    size_t parsed;
    msgpack_zone* z;
    size_t initial_buffer_size;
    void* ctx;
} msgpack_unpacker;


#ifndef MSGPACK_UNPACKER_INIT_BUFFER_SIZE
#define MSGPACK_UNPACKER_INIT_BUFFER_SIZE (64*1024)
#endif

/**
 * Initializes a streaming deserializer.
 * The initialized deserializer must be destroyed by msgpack_unpacker_destroy(msgpack_unpacker*).
 */
MSGPACK_DLLEXPORT
bool msgpack_unpacker_init(msgpack_unpacker* mpac, size_t initial_buffer_size);

/**
 * Destroys a streaming deserializer initialized by msgpack_unpacker_init(msgpack_unpacker*, size_t).
 */
MSGPACK_DLLEXPORT
void msgpack_unpacker_destroy(msgpack_unpacker* mpac);


/**
 * Creates a streaming deserializer.
 * The created deserializer must be destroyed by msgpack_unpacker_free(msgpack_unpacker*).
 */
MSGPACK_DLLEXPORT
msgpack_unpacker* msgpack_unpacker_new(size_t initial_buffer_size);

/**
 * Frees a streaming deserializer created by msgpack_unpacker_new(size_t).
 */
MSGPACK_DLLEXPORT
void msgpack_unpacker_free(msgpack_unpacker* mpac);


#ifndef MSGPACK_UNPACKER_RESERVE_SIZE
#define MSGPACK_UNPACKER_RESERVE_SIZE (32*1024)
#endif

/**
 * Reserves free space of the internal buffer.
 * Use this function to fill the internal buffer with
 * msgpack_unpacker_buffer(msgpack_unpacker*),
 * msgpack_unpacker_buffer_capacity(const msgpack_unpacker*) and
 * msgpack_unpacker_buffer_consumed(msgpack_unpacker*).
 */
static inline bool   msgpack_unpacker_reserve_buffer(msgpack_unpacker* mpac, size_t size);

/**
 * Gets pointer to the free space of the internal buffer.
 * Use this function to fill the internal buffer with
 * msgpack_unpacker_reserve_buffer(msgpack_unpacker*, size_t),
 * msgpack_unpacker_buffer_capacity(const msgpack_unpacker*) and
 * msgpack_unpacker_buffer_consumed(msgpack_unpacker*).
 */
static inline char*  msgpack_unpacker_buffer(msgpack_unpacker* mpac);

/**
 * Gets size of the free space of the internal buffer.
 * Use this function to fill the internal buffer with
 * msgpack_unpacker_reserve_buffer(msgpack_unpacker*, size_t),
 * msgpack_unpacker_buffer(const msgpack_unpacker*) and
 * msgpack_unpacker_buffer_consumed(msgpack_unpacker*).
 */
static inline size_t msgpack_unpacker_buffer_capacity(const msgpack_unpacker* mpac);

/**
 * Notifies the deserializer that the internal buffer filled.
 * Use this function to fill the internal buffer with
 * msgpack_unpacker_reserve_buffer(msgpack_unpacker*, size_t),
 * msgpack_unpacker_buffer(msgpack_unpacker*) and
 * msgpack_unpacker_buffer_capacity(const msgpack_unpacker*).
 */
static inline void   msgpack_unpacker_buffer_consumed(msgpack_unpacker* mpac, size_t size);


/**
 * Deserializes one object.
 * Returns true if it successes. Otherwise false is returned.
 * @param pac  pointer to an initialized msgpack_unpacked object.
 */
MSGPACK_DLLEXPORT
msgpack_unpack_return msgpack_unpacker_next(msgpack_unpacker* mpac, msgpack_unpacked* pac);

/**
 * Initializes a msgpack_unpacked object.
 * The initialized object must be destroyed by msgpack_unpacked_destroy(msgpack_unpacker*).
 * Use the object with msgpack_unpacker_next(msgpack_unpacker*, msgpack_unpacked*) or
 * msgpack_unpack_next(msgpack_unpacked*, const char*, size_t, size_t*).
 */
static inline void msgpack_unpacked_init(msgpack_unpacked* result);

/**
 * Destroys a streaming deserializer initialized by msgpack_unpacked().
 */
static inline void msgpack_unpacked_destroy(msgpack_unpacked* result);

/**
 * Releases the memory zone from msgpack_unpacked object.
 * The released zone must be freed by msgpack_zone_free(msgpack_zone*).
 */
static inline msgpack_zone* msgpack_unpacked_release_zone(msgpack_unpacked* result);


MSGPACK_DLLEXPORT
int msgpack_unpacker_execute(msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
msgpack_object msgpack_unpacker_data(msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
msgpack_zone* msgpack_unpacker_release_zone(msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
void msgpack_unpacker_reset_zone(msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
void msgpack_unpacker_reset(msgpack_unpacker* mpac);

static inline size_t msgpack_unpacker_message_size(const msgpack_unpacker* mpac);


/** @} */


// obsolete
MSGPACK_DLLEXPORT
msgpack_unpack_return
msgpack_unpack(const char* data, size_t len, size_t* off,
        msgpack_zone* result_zone, msgpack_object* result);




static inline size_t msgpack_unpacker_parsed_size(const msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
bool msgpack_unpacker_flush_zone(msgpack_unpacker* mpac);

MSGPACK_DLLEXPORT
bool msgpack_unpacker_expand_buffer(msgpack_unpacker* mpac, size_t size);

static inline bool msgpack_unpacker_reserve_buffer(msgpack_unpacker* mpac, size_t size)
{
    if(mpac->free >= size) { return true; }
    return msgpack_unpacker_expand_buffer(mpac, size);
}

static inline char* msgpack_unpacker_buffer(msgpack_unpacker* mpac)
{
    return mpac->buffer + mpac->used;
}

static inline size_t msgpack_unpacker_buffer_capacity(const msgpack_unpacker* mpac)
{
    return mpac->free;
}

static inline void msgpack_unpacker_buffer_consumed(msgpack_unpacker* mpac, size_t size)
{
    mpac->used += size;
    mpac->free -= size;
}

static inline size_t msgpack_unpacker_message_size(const msgpack_unpacker* mpac)
{
    return mpac->parsed - mpac->off + mpac->used;
}

static inline size_t msgpack_unpacker_parsed_size(const msgpack_unpacker* mpac)
{
    return mpac->parsed;
}


static inline void msgpack_unpacked_init(msgpack_unpacked* result)
{
    memset(result, 0, sizeof(msgpack_unpacked));
}

static inline void msgpack_unpacked_destroy(msgpack_unpacked* result)
{
    if(result->zone != NULL) {
        msgpack_zone_free(result->zone);
        result->zone = NULL;
        memset(&result->data, 0, sizeof(msgpack_object));
    }
}

static inline msgpack_zone* msgpack_unpacked_release_zone(msgpack_unpacked* result)
{
    if(result->zone != NULL) {
        msgpack_zone* z = result->zone;
        result->zone = NULL;
        return z;
    }
    return NULL;
}


#ifdef __cplusplus
}
#endif

#endif /* msgpack/unpack.h */

