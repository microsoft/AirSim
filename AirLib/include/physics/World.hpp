// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_World_hpp
#define airsim_core_World_hpp

#include <functional>
#include "common/Common.hpp"
#include "common/UpdatableContainer.hpp"
#include "PhysicsEngineBase.hpp"
#include "PhysicsBody.hpp"
#include "common/common_utils/ScheduledExecutor.hpp"
#include "common/ClockFactory.hpp"

namespace msr { namespace airlib {

class World : public UpdatableContainer<UpdatableObject*> {
public:
    World(PhysicsEngineBase* physics_engine)
    { 
        World::clear();

        physics_engine_ = physics_engine;
        if (physics_engine)
            physics_engine_->clear();
    }

    //override updatable interface so we can synchronize physics engine
    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        UpdatableContainer::reset();
        
        if (physics_engine_)
            physics_engine_->reset();
    }

    virtual void update() override
    {
        ClockFactory::get()->step();

        //first update our objects
        UpdatableContainer::update();

        //now update kinematics state
        if (physics_engine_)
            physics_engine_->update();
    }

    virtual void reportState(StateReporter& reporter) override
    {
        reporter.writeValue("Sleep", 1.0f / executor_.getSleepTimeAvg());
        if (physics_engine_)
            physics_engine_->reportState(reporter);

        //call base
        UpdatableContainer::reportState(reporter);
    }
    //*** End: UpdatableState implementation ***//

    //override membership modification methods so we can synchronize physics engine
    virtual void clear() override
    { 
        if (physics_engine_)
            physics_engine_->clear();
        UpdatableContainer::clear();
    }

    virtual void insert(UpdatableObject* member) override
    { 
        if (physics_engine_ && member->getPhysicsBody() != nullptr)
            physics_engine_->insert(static_cast<PhysicsBody*>(member->getPhysicsBody()));

        UpdatableContainer::insert(member);
    }

    virtual void erase_remove(UpdatableObject* member) override
    { 
        if (physics_engine_ && member->getPhysicsBody() != nullptr)
            physics_engine_->erase_remove(static_cast<PhysicsBody*>(
                member->getPhysicsBody()));

        UpdatableContainer::erase_remove(member);
    }

    //async updater thread
    void startAsyncUpdator(uint64_t period)
    {
        //TODO: probably we shouldn't be passing around fixed period
        executor_.initialize(std::bind(&World::worldUpdatorAsync, this, std::placeholders::_1), period);
        executor_.start();
    }
    void stopAsyncUpdator()
    {
        executor_.stop();
    }
    void lock()
    {
        executor_.lock();
    }
    void unlock()
    {
        executor_.unlock();
    }

    virtual ~World()
    {
        executor_.stop();
    }

private:
    bool worldUpdatorAsync(uint64_t dt_nanos)
    {
        unused(dt_nanos);

        try {
            update();
        }
        catch(const std::exception& ex) {
            //Utils::DebugBreak();
            Utils::log(Utils::stringf("Exception occurred while updating world: %s", ex.what()), Utils::kLogLevelError);
        }
        catch(...) {
            //Utils::DebugBreak();
            Utils::log("Exception occurred while updating world", Utils::kLogLevelError);
        }

        return true;
    }

private:
    PhysicsEngineBase* physics_engine_ = nullptr;
    common_utils::ScheduledExecutor executor_;
};

}} //namespace
#endif
