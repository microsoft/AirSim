#ifndef msr_AirLibUnitTests_WorkerThreadThreadTest_hpp
#define msr_AirLibUnitTests_WorkerThreadThreadTest_hpp

#include "common/common_utils/WorkerThread.hpp"
#include "common/common_utils/Timer.hpp"
#include <chrono>
#include "TestBase.hpp"

namespace msr { namespace airlib {


class WorkerThreadTest : public TestBase {
    class WorkItem : public CancelableBase {
    public:
        double runTime = 2; // seconds

        virtual void execute() {
            common_utils::Timer timer;
            timer.start();
            while (!this->isCancelled())
            {
                counter_++;
                std::this_thread::sleep_for(std::chrono::microseconds(1));
                if (timer.seconds() > runTime) {
                    break;
                }
            }
        }
        void reset() {
            // reset for next test
            is_cancelled_ = false;
        }
        uint64_t getCount() {
            return counter_;
        }
    private:
        uint64_t counter_;
    };

public:
    virtual void run() override
    {
        WorkerThread thread;

        std::shared_ptr<WorkItem> item1 = std::make_shared<WorkItem>();
        std::shared_ptr<WorkItem> item2 = std::make_shared<WorkItem>();

        thread.enqueue(item1);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        uint64_t item1count = item1->getCount();

        // cancel item1 and switch to item2
        thread.enqueue(item2);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        uint64_t item1count2 = item1->getCount();
        uint64_t item2count = item2->getCount();

        testAssert(item1count > 20, "item1 call count is too low");
        testAssert(item1count2 < 10 + item1count, "canceling item1 took too long");
        testAssert(item2count > 20, "item2 call count is too low");

        // make sure we can cancel.
        thread.cancel();
        uint64_t item2count2 = item2->getCount();
        testAssert(item2count2 < 10 + item2count, "item2 is not canceling");

        // now stress the thread switching...(this found threading bugs in CallLock!)
        for (size_t i = 0; i < 100; i++)
        {
            item1->reset();
            thread.enqueue(item1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            item2->reset();
            thread.enqueue(item2);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        uint64_t item1count3 = item1->getCount();
        uint64_t item2count3 = item2->getCount();

        testAssert(item1count3 > item1count2 + 50, "item1 is getting starved");
        testAssert(item2count3 > item2count2 + 50, "item2 is getting starved");

        // test that enqueueAndWait works.
        item1->reset();
        item1->runTime = 2; // two seconds
        
        common_utils::Timer timer;
        timer.start();
        thread.enqueueAndWait(item1, 0.5);
        double elapsed = timer.seconds();
        testAssert(elapsed >= 0.5 && elapsed < 1 && !item1->isCancelled(), "enqueueAndWait waited too long, should have timed out");

        item2->reset();
        item2->runTime = 0.5; // half a second
        timer.start();
        thread.enqueueAndWait(item2, 2);
        elapsed = timer.seconds();
        testAssert(elapsed >= 0.5 && elapsed < 1 && item2->isCancelled(), "enqueueAndWait waited too long, task should have completed in 0.5 seconds");
    }
};


}}
#endif