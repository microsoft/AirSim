// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// this is done this way so we can hide the boost dependency from our public SDK headers.
#include "MavLinkSemaphore.hpp"
#include "Utils.hpp"

using namespace mavlinkcom;
using namespace common_utils;

#ifdef __APPLE__
#include <signal.h> //SIGALRM
#endif

#ifdef _WIN32
#include <Windows.h>

class MavLinkSemaphore::semaphore_impl
{
	HANDLE semaphore;

public:
	semaphore_impl() {
		this->semaphore = CreateSemaphore(NULL, 0, MAXINT32, NULL);
		if (this->semaphore == NULL) {
			throw std::runtime_error(Utils::stringf("CreateSemaphore failed with errno=%d!\n", GetLastError()));
		}
	}

	~semaphore_impl() {
		CloseHandle(semaphore);
		semaphore = nullptr;
	}

	void post()
	{
		LONG previous;
		if (0 == ReleaseSemaphore(semaphore, 1, &previous)) {
			throw std::runtime_error(Utils::stringf("ReleaseSemaphore failed with errno=%d!\n", GetLastError()));
		}
	}

	// WaitOne indefinitely for one Signal.  If a Signal has already been posted then WaitOne returns immediately
	// decrementing the count so the next WaitOne may block.
	void wait()
	{
		int rc = WaitForSingleObjectEx(semaphore, INFINITE, TRUE);
		if (rc == WAIT_OBJECT_0) {
			return;
		}
		else {
			throw std::runtime_error(Utils::stringf("WaitForSingleObjectEx failed with unexpected errno=%d!\n", GetLastError()));
		}
	}

	bool timed_wait(int milliseconds)
	{
		int rc = WaitForSingleObjectEx(semaphore, milliseconds, TRUE);
		if (rc == WAIT_OBJECT_0) {
			return true;
		}
		else if (rc == WAIT_ABANDONED)
		{
			throw std::runtime_error("Semaphore was destroyed while we are waiting on it.");
		}
		else if (rc != WAIT_TIMEOUT)
		{
			// perhaps we have WAIT_IO_COMPLETION interrupt...
			throw std::runtime_error(Utils::stringf("WaitForSingleObjectEx failed with errno=%d!\n", GetLastError()));
		}
		return false;
	}
};

#else // posix

#include <semaphore.h>

class MavLinkSemaphore::semaphore_impl
{
	sem_t semaphore;
public:

	semaphore_impl() {
		if (sem_init(&semaphore, /* shared */ 0, /* value*/ 0) == -1) {
			throw std::runtime_error(Utils::stringf("sem_init failed with errno=%d!\n", errno));
		}
	}

	~semaphore_impl() {
		sem_destroy(&semaphore);
	}

	void post()
	{
		int status = sem_post(&semaphore);
		if (status < 0)
		{
			throw std::runtime_error(Utils::stringf("sem_post failed with errno=%d!\n", errno));
		}
	}

	void wait()
	{
		int rc = sem_wait(&semaphore);
		if (rc != 0)
		{
			throw std::runtime_error(Utils::stringf("sem_wait failed with unexpected errno=%d!\n", rc));
		}
	}

	bool timed_wait(int milliseconds)
	{
		// convert to absolute time.
		if (milliseconds < 0)
		{
			throw std::runtime_error("cannot wait for negative milliseconds");
		}
		auto absolute = std::chrono::system_clock::now().time_since_epoch() + std::chrono::milliseconds(milliseconds);
		auto seconds = std::chrono::duration_cast<std::chrono::seconds>(absolute);
		absolute = absolute - seconds;
		auto nanoSecondsRemaining = std::chrono::duration_cast<std::chrono::nanoseconds>(absolute);
		struct timespec ts;
		ts.tv_sec = seconds.count(); // seconds
		milliseconds -= (ts.tv_sec * 1000);
		ts.tv_nsec = nanoSecondsRemaining.count(); // nanoseconds
		int rc = sem_timedwait(&semaphore, &ts);
		if (rc != 0)
		{
			rc = errno;
			if (rc == ETIMEDOUT)
			{
				return false;
			}
			else if (rc != EINTR)
			{
				throw std::runtime_error(Utils::stringf("sem_timedwait failed with unexpected errno=%d!\n", rc));
			}
		}
		return rc == 0;
	}

};
#endif


MavLinkSemaphore::MavLinkSemaphore() {

	impl_.reset(new semaphore_impl());
}

MavLinkSemaphore::~MavLinkSemaphore()
{
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
	return impl_->timed_wait(millisecondTimeout);
}

#ifdef __APPLE__
struct CSGX__sem_timedwait_Info
{
    pthread_mutex_t MxMutex;
    pthread_cond_t MxCondition;
    pthread_t MxParent;
    struct timespec MxTimeout;
    bool MxSignaled;
};

void *CSGX__sem_timedwait_Child(void *MainPtr)
{
    CSGX__sem_timedwait_Info *TempInfo = (CSGX__sem_timedwait_Info *)MainPtr;

    pthread_mutex_lock(&TempInfo->MxMutex);

    // Wait until the timeout or the condition is signaled, whichever comes first.
    int Result;
    do
    {
        Result = pthread_cond_timedwait(&TempInfo->MxCondition, &TempInfo->MxMutex, &TempInfo->MxTimeout);
        if (!Result)  break;
    } while (1);
    if (errno == ETIMEDOUT && !TempInfo->MxSignaled)
    {
        TempInfo->MxSignaled = true;
        pthread_kill(TempInfo->MxParent, SIGALRM);
    }

    pthread_mutex_unlock(&TempInfo->MxMutex);

    return NULL;
}

int sem_timedwait(sem_t *sem, const struct timespec *abs_timeout)
{
    // Quick test to see if a lock can be immediately obtained.
    int Result;

    do
    {
        Result = sem_trywait(sem);
        if (!Result)  return 0;
    } while (Result < 0 && errno == EINTR);

    // Since it couldn't be obtained immediately, it is time to shuttle the request off to a thread.
    // Depending on the timeout, this could take longer than the timeout.
    CSGX__sem_timedwait_Info TempInfo;

    pthread_mutex_init(&TempInfo.MxMutex, NULL);
    pthread_cond_init(&TempInfo.MxCondition, NULL);
    TempInfo.MxParent = pthread_self();
    TempInfo.MxTimeout.tv_sec = abs_timeout->tv_sec;
    TempInfo.MxTimeout.tv_nsec = abs_timeout->tv_nsec;
    TempInfo.MxSignaled = false;

    sig_t OldSigHandler = signal(SIGALRM, SIG_DFL);

    pthread_t ChildThread;
    pthread_create(&ChildThread, NULL, CSGX__sem_timedwait_Child, &TempInfo);

    // Wait for the semaphore, the timeout to expire, or an unexpected error condition.
    do
    {
        Result = sem_wait(sem);
        if (Result == 0 || TempInfo.MxSignaled || (Result < 0 && errno != EINTR))  break;
    } while (1);

    // Terminate the thread (if it is still running).
    TempInfo.MxSignaled = true;
    int LastError = errno;

    pthread_mutex_lock(&TempInfo.MxMutex);
    pthread_cond_signal(&TempInfo.MxCondition);
    pthread_mutex_unlock(&TempInfo.MxMutex);
    pthread_join(ChildThread, NULL);
    pthread_cond_destroy(&TempInfo.MxCondition);
    pthread_mutex_destroy(&TempInfo.MxMutex);

    // Restore previous signal handler.
    signal(SIGALRM, OldSigHandler);

    errno = LastError;

    return Result;
}
#endif
