// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// this is done this way so we can hide the boost dependency from our public SDK headers.
#include "MavLinkSemaphore.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

using namespace mavlinkcom;

MavLinkSemaphore::MavLinkSemaphore() {
	impl_ = nullptr;
	impl_ = new boost::interprocess::interprocess_semaphore(0);
}

MavLinkSemaphore::~MavLinkSemaphore()
{
	if (impl_ != nullptr) {
		delete impl_;
		impl_ = nullptr;
	}
}

void MavLinkSemaphore::wait()
{
	impl_->wait();
}

void MavLinkSemaphore::post()
{
	impl_->post();
}

bool MavLinkSemaphore::timed_wait(int millisecondTimeout)
{
	auto boost_time = boost::posix_time::microsec_clock::universal_time();
	boost::posix_time::ptime ptimeout = boost_time + boost::posix_time::milliseconds(millisecondTimeout);
	return impl_->timed_wait(ptimeout);
}
