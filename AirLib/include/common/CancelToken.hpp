// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CancelToken_hpp
#define air_CancelToken_hpp

#include <mutex>
#include <atomic>
#include "common/Common.hpp"
#include "common/common_utils/Utils.hpp"

namespace msr
{
namespace airlib
{

    class CancelToken
    {
    public:
        CancelToken()
            : is_cancelled_(false), is_complete_(false), recursion_count_(0)
        {
        }

        void reset()
        {
            is_cancelled_ = false;
            is_complete_ = false;
            recursion_count_ = 0;
        }

        bool isCancelled() const
        {
            return is_cancelled_;
        }

        void cancel()
        {
            is_cancelled_ = true;
        }

        bool sleep(double secs)
        {
            //We can pass duration directly to sleep_for however it is known that on
            //some systems, sleep_for makes system call anyway even if passed duration
            //is <= 0. This can cause 50us of delay due to context switch.
            if (isCancelled()) {
                return false;
            }

            TTimePoint start = ClockFactory::get()->nowNanos();
            static constexpr std::chrono::duration<double> MinSleepDuration(0);

            while (secs > 0 && !isCancelled() &&
                   ClockFactory::get()->elapsedSince(start) < secs) {

                std::this_thread::sleep_for(MinSleepDuration);
            }

            return !isCancelled();
        }

        void complete(bool is_complete = true)
        {
            is_complete_ = is_complete;
        }

        bool isComplete() const
        {
            return is_complete_;
        }

        int getRecursionCount()
        {
            return recursion_count_;
        }

        bool try_lock()
        {
            bool result = wait_mutex_.try_lock();
            if (result)
                ++recursion_count_;
            return result;
        }

        void unlock()
        {
            wait_mutex_.unlock();
            if (recursion_count_ > 0)
                --recursion_count_;
        }

        void lock()
        {
            wait_mutex_.lock();
            ++recursion_count_;
        }

    private:
        std::atomic<bool> is_cancelled_;
        std::atomic<bool> is_complete_;
        std::atomic<int> recursion_count_;

        std::recursive_mutex wait_mutex_;
    };
}
} //namespace
#endif
