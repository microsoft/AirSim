// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef commn_utils_WorkerThread_hpp
#define commn_utils_WorkerThread_hpp

#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <exception>
#include <future>
#include <mutex>
#include "Timer.hpp"
#include "Utils.hpp"

namespace msr {
    namespace airlib {

        class CancelableBase {
        protected:
            std::atomic<bool> is_cancelled_;
            std::atomic<bool> is_complete_;
        public:
            CancelableBase() : is_cancelled_(false), is_complete_(false) {
            }
            bool isCancelled() {
                return is_cancelled_;
            }

            void cancel() {
                is_cancelled_ = true;
            }

            virtual void execute() = 0;

            bool sleep(double secs)
            {
                //We can pass duration directly to sleep_for however it is known that on 
                //some systems, sleep_for makes system call anyway even if passed duration 
                //is <= 0. This can cause 50us of delay due to context switch.
                if (isCancelled()) {
                    return false;
                }

                common_utils::Timer timer;
                timer.start();

                while(secs > 0 && !isCancelled() && !is_complete_ && timer.seconds() < secs)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));

                return !isCancelled();
            }

            void complete() {
                is_complete_ = true;
            }

            bool isComplete() {
                return is_complete_;
            }

        };

        // This wraps a condition_variable so we can handle the case where we may signal before wait
        // and implement the semantics that say wait should be a noop in that case.
        class WorkerThreadSignal
        {
            std::condition_variable cv_;
            std::mutex mutex_;
            std::atomic<bool> signaled_;
        public:
            WorkerThreadSignal() : signaled_(false) {
            }
            void signal() {
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    signaled_ = true;
                }
                cv_.notify_one();
            }

            template<class _Predicate>
            void wait(_Predicate cancel)
            {
                // wait for signal or cancel predicate
                while (!signaled_) {
                    std::unique_lock<std::mutex> lock(mutex_);
                    cv_.wait_for(lock, std::chrono::milliseconds(1), [this, cancel] {
                        return cancel();
                    });
                }
                signaled_ = false;
            }

            bool waitFor(double max_wait_seconds)
            {
                // wait for signal or timeout or cancel predicate
                while (!signaled_) {
                    std::unique_lock<std::mutex> lock(mutex_);
                    cv_.wait_for(lock, std::chrono::milliseconds(
                        static_cast<long long>(max_wait_seconds * 1000)));
                }
                signaled_ = false;
                return true;
            }

        };

        // This class provides a synchronized worker thread that guarantees to execute
        // cancelable tasks on a background thread one at a time.
        // It guarantees that previous task is cancelled before new task is started.
        // The queue size is 1, which means it does NOT guarantee all queued tasks are executed.
        // If enqueue is called very quickly the thread will not have a chance to execute each
        // task before they get cancelled, worst case in a tight loop all tasks are starved and
        // nothing executes. 
        class WorkerThread {
        public:
            WorkerThread()
            : thread_running_(false), thread_stopped_(false) {
            }

            ~WorkerThread() {
                thread_stopped_ = true;
                cancel();
            }
            void enqueue(std::shared_ptr<CancelableBase> item) {
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    std::shared_ptr<CancelableBase> pending = pending_;
                    if (pending != nullptr) {
                        pending->cancel();
                    }
                }

                bool running = false;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    pending_ = item;
                    running = thread_running_;
                }
                if (running) {
                    thread_waiting_.signal();
                }
                else {
                    start();
                }
            }

            bool enqueueAndWait(std::shared_ptr<CancelableBase> item, float max_wait_seconds) {
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    std::shared_ptr<CancelableBase> pending = pending_;
                    if (pending != nullptr) {
                        pending->cancel();
                    }
                }

                bool running = false;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    pending_ = item;
                    running = thread_running_;
                }
                if (running) {
                    thread_waiting_.signal();
                }
                else {
                    start();
                }

                item->sleep(max_wait_seconds);
                return !item->isCancelled();
            }
            void cancel() {
                std::unique_lock<std::mutex> lock(mutex_);
                std::shared_ptr<CancelableBase> pending = pending_;
                pending_ = nullptr;
                if (pending != nullptr) {
                    pending->cancel();
                }
                if (thread_.joinable()) {
                    thread_waiting_.signal();
                    thread_.join();
                }
            }
        private:
            void start() {
                if (!thread_running_) {
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        if (thread_.joinable()) {
                            thread_.join();
                        }
                    }
                    thread_stopped_ = false;
                    Utils::cleanupThread(thread_);
                    thread_ = std::thread(&WorkerThread::run, this);
                    thread_started_.wait([this] { return static_cast<bool>(thread_stopped_); });
                }
            }
            void run() {
                thread_running_ = true;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    thread_started_.signal();
                }
                while (!thread_stopped_ && pending_ != nullptr) {
                    std::shared_ptr<CancelableBase> pending;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        pending = pending_;
                    }
                    if (pending != nullptr && !pending->isCancelled()) {
                        try {
                            pending->execute();
                        }
                        catch (std::exception& e) {
                            Utils::log(Utils::stringf("WorkerThread caught unhandled exception: %s", e.what()), Utils::kLogLevelError);
                        }
                        // we use cancel here to communicate to enqueueAndWait that the task is complete.
                        pending->complete();
                    }

                    if (!thread_stopped_) {
                        thread_waiting_.wait([this] { return static_cast<bool>(thread_stopped_); });
                    }
                }
                thread_running_ = false;
            }
        private:
            WorkerThreadSignal thread_started_;
            WorkerThreadSignal thread_waiting_;

            // thread state
            std::shared_ptr<CancelableBase> pending_;
            std::mutex mutex_;
            std::thread thread_;
            std::atomic<bool> thread_running_;
            std::atomic<bool> thread_stopped_;
        };

    }
}
#endif