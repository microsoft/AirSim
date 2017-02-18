// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsBody_hpp
#define airsim_core_PhysicsBody_hpp

#include "common/Common.hpp"
#include "common/UpdatableContainer.hpp"
#include "PhysicsBodyVertex.hpp"
#include "common/CommonStructs.hpp"
#include "Kinematics.hpp"
#include "Environment.hpp"
#include <unordered_set>

namespace msr { namespace airlib {

class PhysicsBody : public UpdatableObject {
public: //abstract interface
    virtual Vector3r getLinearDragFactor() const = 0;
    virtual Vector3r getAngularDragFactor() const = 0;
    virtual uint vertexCount() const = 0;
    virtual void kinematicsUpdated(real_T dt) = 0;
    virtual real_T getRestitution() const = 0;
    virtual real_T getFriction() const = 0;
    //derived class may return covariant type
    //TODO: add const version
    virtual PhysicsBodyVertex& getVertex(uint index) = 0;
    virtual const PhysicsBodyVertex& getVertex(uint index) const = 0;

public: //methods
    //constructors
    PhysicsBody()
    {
        PhysicsBody::reset();
    }
    PhysicsBody(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        initialize(mass, inertia, initial_kinematic_state, environment);
    }
    void initialize(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        kinematics_.initialize(initial_kinematic_state);

        mass_ = mass;
        inertia_ = inertia;
        inertia_inv_ = inertia_.inverse();
        environment_ = environment;

        PhysicsBody::reset();
    }

    //enable physics body detection
    virtual UpdatableObject* getPhysicsBody() override
    {
        return this;
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        kinematics_.reset();
        if (environment_)
            environment_->reset();
        wrench_ = Wrench::zero();
    }

    virtual void update(real_T dt) override
    {
        //update position from kinematics so we have latest position after physics update
        environment_->setPosition(getKinematics().pose.position);
        environment_->update(dt);

        kinematics_.update(dt);

        //update individual vertices
        for (uint vertex_index = 0; vertex_index < vertexCount(); ++vertex_index) {
            getVertex(vertex_index).update(dt);
        }
    }
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeHeading("Kinematics");
        kinematics_.reportState(reporter);
    }
    //*** End: UpdatableState implementation ***//


    //getters
    real_T getMass()  const
    {
        return mass_;
    }
    const Matrix3x3r& getInertia()  const
    {
        return inertia_;
    }
    const Matrix3x3r& getInertiaInv()  const
    {
        return inertia_inv_;
    }

    const Pose& getPose() const
    {
        return kinematics_.getPose();
    }
    void setPose(const Pose& pose)
    {
        return kinematics_.setPose(pose);
    }
    const Twist& getTwist() const
    {
        return kinematics_.getTwist();
    }
    void setTwist(const Twist& twist)
    {
        return kinematics_.setTwist(twist);
    }


    const Kinematics::State& getKinematics() const
    {
        return kinematics_.getState();
    }
    void setKinematics(const Kinematics::State& state)
    {
        kinematics_.setState(state);
    }
    const Kinematics::State& getInitialKinematics() const
    {
        return kinematics_.getInitialState();
    }
    const Environment& getEnvironment() const
    {
        return *environment_;
    }
    bool hasEnvironment() const
    {
        return environment_ != nullptr;
    }
    const Wrench& getWrench() const
    {
        return wrench_;
    }
    void setWrench(const Wrench&  wrench)
    {
        wrench_ = wrench;
    }
    const CollisionInfo& getCollisionInfo() const
    {
        return collison_info_;
    }
    //ability to get reference so individual fields can be modified
    void setCollisionInfo(const CollisionInfo& collison_info)
    {
        collison_info_ = collison_info;
    }

private:
    real_T mass_;
    Matrix3x3r inertia_, inertia_inv_;

    Kinematics kinematics_;

    //force is in world frame but torque is not
    //TODO: make torque in world frame too
    Wrench wrench_;

    CollisionInfo collison_info_;

    Environment* environment_ = nullptr;
};

}} //namespace
#endif
