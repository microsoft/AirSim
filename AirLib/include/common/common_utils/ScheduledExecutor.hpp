// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef commn_utils_ScheduledExecutor_hpp
#define commn_utils_ScheduledExecutor_hpp

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <system_error>
#include <mutex>


namespace common_utils {

class ScheduledExecutor {
public:
    ScheduledExecutor()
    {}
    ScheduledExecutor(const std::function<bool(double)>& callback, double period)
    {
        initialize(callback, period);
    }
    ~ScheduledExecutor()
    {
        stop();
    }
    void initialize(const std::function<bool(double)>& callback, double period)
    {
        callback_ = callback;
        period_ = period;
        keep_running_ = false;
    }

    void start()
    {
        keep_running_ = true;
        sleep_time_avg_ = 0;
        period_count_ = 0;
        th_ = std::thread(&ScheduledExecutor::executorLoop, this);
    }

    void stop()
    {
        if (keep_running_) {
            keep_running_ = false;
            try {
                if (th_.joinable()) {
                    th_.join();
                }
            }
            catch(const std::system_error& /* e */)
            { }
        }
    }

    bool isRunning()
    {
        return keep_running_;
    }

    double getSleepTimeAvg()
    {
        //TODO: make this function thread safe by using atomic types
        //right now this is not implemented for performance and that
        //return of this function is purely informational/debugging purposes
        return sleep_time_avg_;
    }

    unsigned long getPeriodCount()
    {
        return period_count_;
    }

    void lock()
    {
        mutex_.lock();
    }
    void unlock()
    {
        mutex_.unlock();
    }

private:
    typedef std::chrono::high_resolution_clock clock;
    template <typename T>
    using duration = std::chrono::duration<T>;

    static void sleep_for(double dt)
    {
        /*
        This is spin loop implementation which may be suitable for sub-millisecond resolution.
        //TODO: investigate below alternatives
        On Windows we can use multimedia timers however this requires including entire Win32 header.
        On Linux we can use nanosleep however below 2ms delays in real-time scheduler settings this 
        probbaly does spin loop anyway.

        */
        static constexpr duration<double> MinSleepDuration(0);
        clock::time_point start = clock::now();
        while (duration<double>(clock::now() - start).count() < dt) {
            std::this_thread::sleep_for(MinSleepDuration);
        }
    }

    void executorLoop()
    {
        clock::time_point call_end = clock::now();
        while (keep_running_) {
            clock::time_point period_start = clock::now();
            duration<double> since_last_call = period_start - call_end;
            
            if (period_count_ > 0) {
                bool result = callback_(since_last_call.count());
                if (!result) {
                    keep_running_ = result;
                }
            }
            
            call_end = clock::now();

            //after running callback, try to get lock which might cause wait
            {
                //TODO: add ability to disable use of mutex
                std::lock_guard<std::mutex> locker(mutex_);

                duration<double> elapsed_period = clock::now() - period_start;
                double sleep_dt = period_ - elapsed_period.count();
                sleep_time_avg_ = 0.25f * sleep_time_avg_ + 0.75f * sleep_dt;
                ++period_count_;
                if (sleep_dt > MinSleepTime && keep_running_)
                    sleep_for(sleep_dt);
                    //std::this_thread::sleep_for(std::chrono::duration<double>(sleep_dt));
            }
        }
    }

private:
    double period_;
    std::thread th_;
    std::function<bool(double)> callback_;
    std::atomic_bool keep_running_;
    
    static constexpr double MinSleepTime = 1E-6;

    double sleep_time_avg_;
    unsigned long period_count_;

    std::mutex mutex_;
};

}
#endif
