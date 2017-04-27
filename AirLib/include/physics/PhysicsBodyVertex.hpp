// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsBodyVertex_hpp
#define airsim_core_PhysicsBodyVertex_hpp

#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class PhysicsBodyVertex : public UpdatableObject {
protected:
    virtual void setWrench(Wrench& wrench) = 0;
public:
    PhysicsBodyVertex()
    {
        //allow default constructor with later call for initialize
    }
    PhysicsBodyVertex(const Vector3r& position, const Vector3r& normal)
    {
        initialize(position, normal);
    }
    void initialize(const Vector3r& position, const Vector3r& normal)
    {
        initial_position_ = position;
        initial_normal_ = normal;

        PhysicsBodyVertex::reset();
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        last_time_ = clock()->nowNanos();
        
        position_ = initial_position_;
        normal_ = initial_normal_;

        current_wrench_ = Wrench::zero();
    }

    virtual void update() override
    {
        TTimeDelta dt = clock()->updateSince(last_time_);
        
        setWrench(current_wrench_);
    }
    //*** End: UpdatableState implementation ***//


    //getters, setters
    Vector3r getPosition() const
    {
        return position_;
    }
    void setPosition(const Vector3r& position)
    {
        position_ = position;
    }


    Vector3r getNormal() const
    {
        return normal_;
    }
    void setNormal(const Vector3r& normal)
    {
        normal_ = normal;
    }


    Wrench getWrench() const
    {
        return current_wrench_;
    }

private:
    Vector3r initial_position_, position_;
    Vector3r initial_normal_, normal_;
    Wrench current_wrench_;
    TTimePoint last_time_;
};

}} //namespace
#endif
