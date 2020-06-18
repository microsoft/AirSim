// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/BufferPool.h"

template <typename T>
typename BufferPool<T>::BufferPtr BufferPool<T>::GetBufferExactSize(size_t size)
{
    if (CollectionsBySize.count(size) == 0)
        CollectionsBySize.insert(std::pair<size_t, BufferCollection>(size, BufferCollection(size)));

    return CollectionsBySize.at(size).DemandBuffer();
}

template <typename T>
typename BufferPool<T>::BufferPtr BufferPool<T>::GetBufferAtLeastSize(size_t size, size_t max_size)
{
    BufferPtr buffer = nullptr;
    auto closestOffering = CollectionsBySize.lower_bound(size);
    if (closestOffering != CollectionsBySize.end() && closestOffering->first < max_size)
        buffer = closestOffering->second.GetBufferIfAvailable();
    
    if (buffer != nullptr)
        return buffer;
    return GetBufferExactSize(size);
}

template <typename T>
typename BufferPool<T>::BufferPtr BufferPool<T>::BufferCollection::DemandBuffer()
{
    BufferPtr buf = GetBufferIfAvailable();
    if (buf != nullptr)
        return buf;
    return MakeBufferPtr();
}

template <typename T>
typename BufferPool<T>::BufferPtr BufferPool<T>::BufferCollection::GetBufferIfAvailable()
{
    if (AvailableBuffers.size() == 0)
        return nullptr;

    Buffer *buffer = *AvailableBuffers.begin();
    AvailableBuffers.erase(buffer);
    return MakeBufferPtr(buffer);
}

template <typename T>
typename BufferPool<T>::BufferPtr BufferPool<T>::BufferCollection::MakeBufferPtr(Buffer *underlyingBuffer)
{
    if (underlyingBuffer == nullptr)
        return std::unique_ptr<Buffer, Deleter>(new Buffer(Size), std::bind(&BufferPool::BufferCollection::ReturnToCollection, this, std::placeholders::_1));
    else
        return std::unique_ptr<Buffer, Deleter>(underlyingBuffer, std::bind(&BufferPool::BufferCollection::ReturnToCollection, this, std::placeholders::_1));
}

template <typename T>
void BufferPool<T>::BufferCollection::ReturnToCollection(Buffer *buffer)
{
    AvailableBuffers.insert(buffer);
}