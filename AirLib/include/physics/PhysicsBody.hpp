// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsBody_hpp
#define airsim_core_PhysicsBody_hpp

#include "common/Common.hpp"
#include "common/UpdatableObject.hpp"
#include "PhysicsBodyVertex.hpp"
#include "common/CommonStructs.hpp"
#include "Kinematics.hpp"
#include "Environment.hpp"
#include <unordered_set>
#include <exception>

namespace msr { namespace airlib {

class PhysicsBody : public UpdatableObject {
public: //interface
    virtual void kinematicsUpdated() = 0;
    virtual real_T getRestitution() const = 0;
    virtual real_T getFriction() const = 0;
    //derived class may return covariant type
    virtual uint wrenchVertexCount() const
    {
        return 0;
    }
    virtual PhysicsBodyVertex& getWrenchVertex(uint index)
    {
        unused(index);
        throw std::out_of_range("no physics vertex are available");
    }
    virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const
    {
        unused(index);
        throw std::out_of_range("no physics vertex are available");
    }

    virtual uint dragVertexCount() const
    {
        return 0;
    }
    virtual PhysicsBodyVertex& getDragVertex(uint index)
    {
        unused(index);
        throw std::out_of_range("no physics vertex are available");
    }
    virtual const PhysicsBodyVertex& getDragVertex(uint index) const
    {
        unused(index);
        throw std::out_of_range("no physics vertex are available");
    }
    void setWrench(const Wrench&  wrench)
    {
        wrench_ = wrench;
    }

public: //methods
    //constructors
    PhysicsBody()
    {
        //allow default constructor with later call for initialize
    }
    PhysicsBody(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        initialize(mass, inertia, initial_kinematic_state, environment);
    }
    void initialize(real_T mass, const Matrix3x3r& inertia, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        kinematics_.initialize(initial_kinematic_state);

        mass_ = mass;
        mass_inv_ = 1.0f / mass;
        inertia_ = inertia;
        inertia_inv_ = inertia_.inverse();
        environment_ = environment;
    }

    //enable physics body detection
    virtual UpdatableObject* getPhysicsBody() override
    {
        return this;
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        UpdatableObject::reset();

        kinematics_.reset();

        if (environment_)
            environment_->reset();
        wrench_ = Wrench::zero();
        collison_info_ = CollisionInfo();
        collison_response_info_ = CollisonResponseInfo();

        //update individual vertices
        for (uint vertex_index = 0; vertex_index < wrenchVertexCount(); ++vertex_index) {
            getWrenchVertex(vertex_index).reset();
        }
        for (uint vertex_index = 0; vertex_index < dragVertexCount(); ++vertex_index) {
            getDragVertex(vertex_index).reset();
        }
    }

    virtual void update() override
    {
        UpdatableObject::update();

        //update position from kinematics so we have latest position after physics update
        environment_->setPosition(getKinematics().pose.position);
        environment_->update();

        kinematics_.update();

        //update individual vertices
        for (uint vertex_index = 0; vertex_index < wrenchVertexCount(); ++vertex_index) {
            getWrenchVertex(vertex_index).update();
        }
        for (uint vertex_index = 0; vertex_index < dragVertexCount(); ++vertex_index) {
            getDragVertex(vertex_index).update();
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
    real_T getMassInv()  const
    {
        return mass_inv_;
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
        if (VectorMath::hasNan(state.twist.linear))
            Utils::DebugBreak();

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

    const CollisionInfo& getCollisionInfo() const
    {
        return collison_info_;
    }
    
    virtual void setCollisionInfo(const CollisionInfo& collison_info)
    {
        collison_info_ = collison_info;
    }

    const CollisonResponseInfo& getCollisionResponseInfo() const
    {
        return collison_response_info_;
    }
    CollisonResponseInfo& getCollisionResponseInfo()
    {
        return collison_response_info_;
    }


public:
    //for use in physics angine: //TODO: use getter/setter or friend method?
    TTimePoint last_kinematics_time;

private:
    real_T mass_, mass_inv_;
    Matrix3x3r inertia_, inertia_inv_;

    Kinematics kinematics_;

    //force is in world frame but torque is not
    Wrench wrench_;

    CollisionInfo collison_info_;
    CollisonResponseInfo collison_response_info_;

    Environment* environment_ = nullptr;
};

}} //namespace
#endif
