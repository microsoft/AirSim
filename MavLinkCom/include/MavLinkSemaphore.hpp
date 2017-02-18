// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_Semaphore_hpp
#define MavLinkCom_Semaphore_hpp

namespace boost {
	namespace interprocess {
		class interprocess_semaphore;
	}
}
namespace mavlinkcom
{
	class MavLinkSemaphore
	{
	public:
		MavLinkSemaphore();
		~MavLinkSemaphore();
		void wait();
		void post();
		bool timed_wait(int milliseconds);
	private:
		boost::interprocess::interprocess_semaphore* impl_;
	};
}

#endif
