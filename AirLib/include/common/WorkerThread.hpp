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
#include "common/common_utils/Utils.hpp"
#include "ClockFactory.hpp" //TODO: move this out of common_utils
#include "CancelToken.hpp"

namespace msr { namespace airlib {

class CancelableAction : public CancelToken {
protected:
    virtual void executeAction() = 0;

public:
    CancelableAction()
        : is_complete_(false)
    {
    }
    virtual ~CancelableAction();

    void reset()
    {
        is_complete_ = false;
    }

    void execute()
    {
        try {
            executeAction();
            is_complete_ = true;
        }
        catch (...) {
            is_complete_ = false;
            throw;
        }
    }

    bool isComplete()  const
    {
        return is_complete_;
    }

private:
    std::atomic<bool> is_complete_;

};

// This wraps a condition_variable so we can handle the case where we may signal before wait
// and implement the semantics that say wait should be a noop in that case.
class WorkerThreadSignal
{
    std::condition_variable cv_;
    std::mutex mutex_;
    std::atomic<bool> signaled_;
public:
    WorkerThreadSignal() : signaled_(false) 
    {
    }

    void signal() 
    {
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

    bool waitFor(double timeout_sec)
    {
        // wait for signal or timeout or cancel predicate
        while (!signaled_) {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(
                static_cast<long long>(timeout_sec * 1000)));
        }
        signaled_ = false;
        return true;
    }

    void wait()
    {
        // wait for signal or timeout or cancel predicate
        std::unique_lock<std::mutex> lock(mutex_);
        while (!signaled_) {
            cv_.wait(lock);
        }
        lock.unlock();
        signaled_ = false;
    }

    bool waitForRetry(double timeout_sec, int n_times)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!signaled_ && n_times > 0) {
            cv_.wait_for(lock, std::chrono::milliseconds(static_cast<long long>(timeout_sec * 1000)));
            --n_times;
        }
        lock.unlock();
        if (n_times == 0 && !signaled_) {
            return false;
        }
        else {
            signaled_ = false;
            return true;
        }
    }

};

// This class provides a synchronized worker thread that guarantees to execute
// cancelable tasks on a background thread one at a time.
// It guarantees that previous task is canceled before new task is started.
// The queue size is 1, which means it does NOT guarantee all queued tasks are executed.
// If enqueue is called very quickly the thread will not have a chance to execute each
// task before they get canceled, worst case in a tight loop all tasks are starved and
// nothing executes. 
class WorkerThread {
public:
    WorkerThread()
    : thread_running_(false), cancel_request_(false) {
    }

    ~WorkerThread() {
        cancel_request_ = true;
        cancel();
    }
    void enqueue(std::shared_ptr<CancelableAction> item) {
        //cancel previous item
        {
            std::unique_lock<std::mutex> lock(mutex_);
            std::shared_ptr<CancelableAction> pending = pending_item_;
            if (pending != nullptr) {
                pending->cancel();
            }
        }

        bool running = false;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            pending_item_ = item;
            running = thread_running_;
        }

        if (running) {
            item_arrived_.signal();
        }
        else {
            start();
        }
    }

    bool enqueueAndWait(std::shared_ptr<CancelableAction> item, float timeout_sec) 
    {
        //cancel previous item
        {
            std::unique_lock<std::mutex> lock(mutex_);
            std::shared_ptr<CancelableAction> pending = pending_item_;
            if (pending != nullptr) {
                pending->cancel();
            }
        }

        bool running = false;

        //set new item to run
        {
            std::unique_lock<std::mutex> lock(mutex_);
            pending_item_ = item;
            running = thread_running_;
        }

        if (running) {
            item_arrived_.signal();
        }
        else {
            start();
        }

        item->sleep(timeout_sec);

        //after the wait if item is still running then cancel it
        if (!item->isCancelled() && !item->isComplete())
            item->cancel();

        return !item->isCancelled();
    }

    void cancel() 
    {
        std::unique_lock<std::mutex> lock(mutex_);
        std::shared_ptr<CancelableAction> pending = pending_item_;
        pending_item_ = nullptr;
        if (pending != nullptr) {
            pending->cancel();
        }
        if (thread_.joinable()) {
            item_arrived_.signal();
            thread_.join();
        }
    }
private:
    void start() 
    {
        //if state == not running
        if (!thread_running_) {

            //make sure C++ previous thread is done
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (thread_.joinable()) {
                    thread_.join();
                }
            }
            Utils::cleanupThread(thread_);

            //start the thread
            cancel_request_ = false;
            thread_ = std::thread(&WorkerThread::run, this);

            //wait until thread tells us it has started
            thread_started_.wait([this] { return static_cast<bool>(cancel_request_); });
        }
    }

    void run() 
    {
        thread_running_ = true;

        //tell the thread which started this thread that we are on now
        {
            std::unique_lock<std::mutex> lock(mutex_);
            thread_started_.signal();
        }

        //until we don't get stopped and have work to do, keep running
        while (!cancel_request_ && pending_item_ != nullptr) {
            std::shared_ptr<CancelableAction> pending;

            //get the pending item
            {
                std::unique_lock<std::mutex> lock(mutex_);
                pending = pending_item_;
            }

            //if pending item is not yet cancelled
            if (pending != nullptr && !pending->isCancelled()) {

                //execute pending item
                try {
                    pending->execute();
                }
                catch (std::exception& e) {
                    //Utils::DebugBreak();
                    Utils::log(Utils::stringf("WorkerThread caught unhandled exception: %s", e.what()), Utils::kLogLevelError);
                }
            }

            if (!cancel_request_) {
                //wait for next item to arrive or thread is stopped
                item_arrived_.wait([this] { return static_cast<bool>(cancel_request_); });
            }
        }

        thread_running_ = false;
    }

private:
    //this is used to wait until our thread actually gets started
    WorkerThreadSignal thread_started_;
    //when new item arrived, we signal this so waiting thread can continue
    WorkerThreadSignal item_arrived_;

    // thread state
    std::shared_ptr<CancelableAction> pending_item_;
    std::mutex mutex_;
    std::thread thread_;
    //while run() is in progress this is true
    std::atomic<bool> thread_running_;
    //has request to stop this worker thread made?
    std::atomic<bool> cancel_request_;
};


}} //namespace
#endif