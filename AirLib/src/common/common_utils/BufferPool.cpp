// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/BufferPool.h"

BufferPtr BufferPool::GetBufferExactSize(size_t size)
{
	if (CollectionsBySize.count(size) == 0)
		CollectionsBySize.insert(std::pair<size_t, BufferCollection>(size, BufferCollection(size)));

	return CollectionsBySize.at(size).DemandBuffer();
}

BufferPtr BufferPool::GetBufferAtLeastSize(size_t size, size_t max_size)
{
	BufferPtr buffer = nullptr;
	auto closestOffering = CollectionsBySize.lower_bound(size);
	if (closestOffering != CollectionsBySize.end() && closestOffering->first < max_size)
		buffer = closestOffering->second.GetBufferIfAvailable();
	
	if (buffer != nullptr)
		return buffer;
	return GetBufferExactSize(size);
}

BufferPtr BufferPool::BufferCollection::DemandBuffer()
{
	BufferPtr buf = GetBufferIfAvailable();
	if (buf != nullptr)
		return buf;
	return MakeBufferPtr();
}

BufferPtr BufferPool::BufferCollection::GetBufferIfAvailable()
{
	if (AvailableBuffers.size() == 0)
		return nullptr;

	Buffer *buffer = *AvailableBuffers.begin();
	AvailableBuffers.erase(buffer);
	return MakeBufferPtr(buffer);
}

BufferPtr BufferPool::BufferCollection::MakeBufferPtr(Buffer *underlyingBuffer)
{
	if (underlyingBuffer == nullptr)
		return std::unique_ptr<Buffer, Deleter>(new Buffer(Size), std::bind(&BufferPool::BufferCollection::ReturnToCollection, this, std::placeholders::_1));
	else
		return std::unique_ptr<Buffer, Deleter>(underlyingBuffer, std::bind(&BufferPool::BufferCollection::ReturnToCollection, this, std::placeholders::_1));
}

void BufferPool::BufferCollection::ReturnToCollection(Buffer *buffer)
{
	AvailableBuffers.insert(buffer);
}