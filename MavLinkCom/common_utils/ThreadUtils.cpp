#include "ThreadUtils.hpp"

#include <codecvt>

#ifdef _WIN32
#include <windows.h> // SetThreadPriority and GetCurrentThread
#else
#include <pthread.h>
#include <sys/prctl.h>
#endif

using namespace mavlink_utils;

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

typedef HRESULT (WINAPI *SetThreadDescriptionFunction)( _In_ HANDLE hThread, _In_ PCWSTR lpThreadDescription);
static SetThreadDescriptionFunction setThreadDescriptionFunction = nullptr;

bool CurrentThread::setThreadName(const std::string& name)
{
#ifdef _WIN32
    // unfortunately this is only available on Windows 10, and AirSim is not limited to that.
    if (setThreadDescriptionFunction == nullptr) {
        HINSTANCE hGetProcIDDLL = LoadLibrary(L"Kernel32");
        FARPROC func = GetProcAddress(hGetProcIDDLL, "SetThreadDescription");
        if (func != nullptr)
        {
            setThreadDescriptionFunction = (SetThreadDescriptionFunction)func;
        }
    }
    if (setThreadDescriptionFunction != nullptr) {
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        std::wstring wide_path = converter.from_bytes(name.c_str());
        return S_OK == (*setThreadDescriptionFunction)(GetCurrentThread(), wide_path.c_str());
    }
    return false;
#else
    
    return 0 == prctl(PR_SET_NAME, name.c_str(), 0, 0, 0);

#endif

}
