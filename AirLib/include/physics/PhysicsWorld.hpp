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

class PhysicsWorld {
public:
    PhysicsWorld(PhysicsEngineBase* physics_engine, const std::vector<UpdatableObject*>& bodies,
            long long update_period_us = 3000000LL, bool state_reporter_enabled = false,
            bool start_async_updator = true
        )
    {
        enableStateReport(state_reporter_enabled);
        update_period_us_ = update_period_us;
        initializeWorld(physics_engine, bodies, start_async_updator);
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

    void startAsyncUpdator()
    {
        world_.startAsyncUpdator(update_period_us_);
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

    std::string getReport()
    {
        return reporter_.getOutput();
    }

private:
    void initializeWorld(PhysicsEngineBase* physics_engine, 
        const std::vector<UpdatableObject*>& bodies, bool start_async_updator)
    {
        world_.initialize(physics_engine);

        reporter_.initialize(false);
        world_.insert(&reporter_);

        for(size_t bi = 0; bi < bodies.size(); bi++)
            world_.insert(bodies.at(bi));

        world_.reset();

        if (start_async_updator)
            world_.startAsyncUpdator(update_period_us_);
    }

private:
    std::vector<UpdatableObject*> bodies_;
    StateReporterWrapper reporter_;
    World world_;
    long long update_period_us_;
};


}} //namespace
#endif
