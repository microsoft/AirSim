#include "ThreadUtils.hpp"


#ifdef _WIN32
#include <windows.h> // SetThreadPriority and GetCurrentThread
#else
#include <pthread.h>
#endif

using namespace common_utils;

// make the current thread run with maximum priority.
bool CurrentThread::setMaximumPriority()
{
#ifdef _WIN32
	HANDLE thread = GetCurrentThread();
    // THREAD_PRIORITY_HIGHEST is too high and makes animation a bit jumpy.
	int rc = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
	if (rc == 0) {
		rc = GetLastError();
		return false;
	}
	return true;
#elif defined(__APPLE__)
    // TODO: How to handle POSIX thread priorities on OSX?
    return true;
#else
	int policy;
	struct sched_param param;
	int err = pthread_getschedparam(pthread_self(), &policy, &param);
	if (err != 0) return false;
	int maxPriority = sched_get_priority_max(policy);
	err = pthread_setschedprio(pthread_self(), maxPriority);
	return err == 0;
#endif
}
