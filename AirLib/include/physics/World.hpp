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

namespace msr { namespace airlib {

class World : public UpdatableContainer<UpdatableObject*> {
public:
    World()
    { 
        initialize(nullptr);
    }
    World(PhysicsEngineBase* physics_engine)
    {
        initialize(physics_engine);
    }
    void initialize(PhysicsEngineBase* physics_engine)
    {
        World::clear();

        if (physics_engine) {
            physics_engine_ = physics_engine;
            physics_engine_->clear();
        }
        World::reset();
    }

    //override updatable interface so we can synchronize physics engine
    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        UpdatableContainer::reset();
        
        if (physics_engine_)
            physics_engine_->reset();
    }

    virtual void update(real_T dt) override
    {
        //first update our objects
        UpdatableContainer::update(dt);

        //now update kinematics state
        if (physics_engine_)
            physics_engine_->update(dt);
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

    virtual void insert(UpdatableObject* member) 
    { 
        if (physics_engine_ && member->getPhysicsBody() != nullptr)
            physics_engine_->insert(static_cast<PhysicsBody*>(member->getPhysicsBody()));

        UpdatableContainer::insert(member);
    }

    virtual void erase_remove(UpdatableObject* member) 
    { 
        if (physics_engine_ && member->getPhysicsBody() != nullptr)
            physics_engine_->erase_remove(static_cast<PhysicsBody*>(member));

        UpdatableContainer::erase_remove(member);
    }

    //async updater thread
    void startAsyncUpdator(real_T period)
    {
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

private:
    bool worldUpdatorAsync(double dt)
    {
        try {
            update(static_cast<real_T>(dt));
        }
        catch(const std::exception& ex) {
            Utils::logError("Exception occurred while updating world: %s", ex.what());
        }
        catch(...) {
            Utils::logError("Exception occurred while updating world");
        }
        return true;
    }

private:
    PhysicsEngineBase* physics_engine_ = nullptr;

    common_utils::ScheduledExecutor executor_;
};

}} //namespace
#endif
