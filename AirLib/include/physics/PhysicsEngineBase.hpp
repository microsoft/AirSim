// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsEngineBase_hpp
#define airsim_core_PhysicsEngineBase_hpp

#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "PhysicsBody.hpp"

namespace msr { namespace airlib {

class PhysicsEngineBase : public UpdatableContainer<PhysicsBody*> {
public:
    //force derived classes to define this
    virtual void reset() override = 0;
    virtual void update(real_T dt) override = 0;

    virtual void reportState(StateReporter& reporter) override
    {
        //default nothing to report for physics engine
    }
};


}} //namespace
#endif
