// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_ExternalPhysicsEngine_hpp
#define airsim_core_ExternalPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>

namespace msr
{
namespace airlib
{

    class ExternalPhysicsEngine : public PhysicsEngineBase
    {
    public:
        ExternalPhysicsEngine()
        {
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
        }

        virtual void update() override
        {
            PhysicsEngineBase::update();

            for (PhysicsBody* body_ptr : *this) {
                body_ptr->updateKinematics();
                body_ptr->update();
            }
        }
        virtual void reportState(StateReporter& reporter) override
        {
            for (PhysicsBody* body_ptr : *this) {
                reporter.writeValue("ExternalPhysicsEngine", true);
                reporter.writeValue("Is Grounded", body_ptr->isGrounded());
            }
            //call base
            UpdatableObject::reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//
    };

} //namespace
} //namespace
#endif
