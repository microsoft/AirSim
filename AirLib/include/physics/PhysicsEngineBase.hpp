// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsEngineBase_hpp
#define airsim_core_PhysicsEngineBase_hpp

#include "common/UpdatableContainer.hpp"
#include "common/Common.hpp"
#include "PhysicsBody.hpp"

namespace msr
{
namespace airlib
{

    class PhysicsEngineBase : public UpdatableContainer<PhysicsBody*>
    {
    public:
        virtual void update() override
        {
            UpdatableObject::update();
        }

        virtual void reportState(StateReporter& reporter) override
        {
            unused(reporter);
            //default nothing to report for physics engine
        }

        virtual void setWind(const Vector3r& wind) { unused(wind); };
        virtual void setExtForce(const Vector3r& ext_force) { unused(ext_force); };
    };
}
} //namespace
#endif
