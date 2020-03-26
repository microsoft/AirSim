// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once
#include <vector>
#include <map>
#include <memory>
#include <unordered_set>
#include <functional>
#include <limits>

typedef std::vector<uint8_t> Buffer;
typedef std::function<void(Buffer*)> Deleter;
typedef std::unique_ptr<Buffer, Deleter> BufferPtr;

class BufferPool
{
public:
    BufferPtr GetBufferExactSize(size_t size);
    BufferPtr GetBufferAtLeastSize(size_t size, size_t max_size = std::numeric_limits<size_t>::max());

private:
    class BufferCollection
    {
    public:
        BufferCollection(size_t size) : Size(size) {}
        BufferPtr DemandBuffer();
        BufferPtr GetBufferIfAvailable();

    private:
        size_t Size;
        std::unordered_set<Buffer*> AvailableBuffers;
        BufferPtr MakeBufferPtr(Buffer *underlyingBuffer = nullptr);
        void ReturnToCollection(Buffer *buffer);
    };
    std::map<size_t, BufferCollection> CollectionsBySize;
};