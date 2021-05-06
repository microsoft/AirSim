// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsVehicleWorld_hpp
#define airsim_core_PhysicsVehicleWorld_hpp

#include "common/UpdatableContainer.hpp"
#include "common/Common.hpp"
#include "PhysicsBody.hpp"
#include "PhysicsEngineBase.hpp"
#include "World.hpp"
#include "common/StateReporterWrapper.hpp"

namespace msr { namespace airlib {

class PhysicsWorld : UpdatableObject 
{
public:
    PhysicsWorld(std::unique_ptr<PhysicsEngineBase> physics_engine, const std::vector<UpdatableObject*>& bodies,
            uint64_t update_period_nanos = 3000000LL, bool state_reporter_enabled = false,
            bool start_async_updator = true
        )
        : world_(std::move(physics_engine))
    {
        setName("PhysicsWorld");
        enableStateReport(state_reporter_enabled);
        update_period_nanos_ = update_period_nanos;
        initializeWorld(bodies, start_async_updator);
    }

    void lock()
    {
        world_.lock();
    }

    void unlock()
    {
        world_.unlock();
    }

    void reset()
    {
        lock();
        world_.reset();
        unlock();
    }

    void addBody(UpdatableObject* body)
    {
        lock();
        world_.insert(body);
        unlock();
    }

    uint64_t getUpdatePeriodNanos() const
    {
        return update_period_nanos_;
    }

    void startAsyncUpdator()
    {
        world_.startAsyncUpdator(update_period_nanos_);
    }
    void stopAsyncUpdator()
    {
        world_.stopAsyncUpdator();
    }


    void enableStateReport(bool is_enabled)
    {
        reporter_.setEnable(is_enabled);
    }

    void updateStateReport()
    {
        if (reporter_.canReport()) {
            reporter_.clearReport();
            world_.reportState(*reporter_.getReporter());
        }
    }

    std::string getDebugReport()
    {
        return reporter_.getOutput();
    }

    void pause(bool is_paused)
    {
        world_.pause(is_paused);
    }

    bool isPaused() const
    {
        return world_.isPaused();
    }

    void continueForTime(double seconds)
    {
        world_.continueForTime(seconds);
    }

    void continueForFrames(uint32_t frames)
    {
        world_.continueForFrames(frames);
    }

    void setFrameNumber(uint32_t frameNumber)
    {
        world_.setFrameNumber(frameNumber);
    }

    void resetImplementation() override {}

private:
    void initializeWorld(const std::vector<UpdatableObject*>& bodies, bool start_async_updator)
    {
        reporter_.initialize(false);
        world_.insert(&reporter_);

        for(size_t bi = 0; bi < bodies.size(); bi++)
            world_.insert(bodies.at(bi));

        world_.reset();

        if (start_async_updator)
            world_.startAsyncUpdator(update_period_nanos_);
    }

private:
    std::vector<UpdatableObject*> bodies_;
    StateReporterWrapper reporter_;
    World world_;
    uint64_t update_period_nanos_;
};


}} //namespace
#endif
