// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_FastPhysicsEngine_hpp
#define airsim_core_FastPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include "common/CommonStructs.hpp"

namespace msr { namespace airlib {

class FastPhysicsEngine : public PhysicsEngineBase {
public:
    FastPhysicsEngine()
    { 
        FastPhysicsEngine::reset();
    }


    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        for (PhysicsBody* body_ptr : *this) {
            initPhysicsBody(body_ptr);
        }
    }

    virtual void insert(PhysicsBody* body_ptr) override
    {
        PhysicsEngineBase::insert(body_ptr);

        initPhysicsBody(body_ptr);
    }

    virtual void update() override
    {
        for (PhysicsBody* body_ptr : *this) {
            updatePhysics(*body_ptr);
        }
    }
    virtual void reportState(StateReporter& reporter) override
    {
        for (PhysicsBody* body_ptr : *this) {
            reporter.writeValue("Phys", debug_string_.str());
            reporter.writeValue("Is Gounded", grounded_);
            reporter.writeValue("Force (world)", body_ptr->getWrench().force);
            reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
        }
        //call base
        UpdatableObject::reportState(reporter);
    }
    //*** End: UpdatableState implementation ***//

private:
    void initPhysicsBody(PhysicsBody* body_ptr)
    {
        body_ptr->last_kinematics_time = clock()->nowNanos();
    }

    void updatePhysics(PhysicsBody& body)
    {
        TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);

        //get current kinematics state of the body - this state existed since last dt seconds
        const Kinematics::State& current = body.getKinematics();
        Kinematics::State next;
        Wrench next_wrench;

        getNextKinematicsNoCollison(dt, body, current, next, next_wrench);

        if (!getNextKinematicsOnCollison(dt, body, current, next, next_wrench))
            getNextKinematicsOnGround(dt, body, current, next, next_wrench);
        
        body.setKinematics(next);
        body.setWrench(next_wrench);
        body.kinematicsUpdated();
    }

    bool getNextKinematicsOnCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        static constexpr uint kCollisionResponseCycles = 1;

        /************************* Collison response ************************/
        const CollisionInfo collison_info = body.getCollisionInfo();
        //if there is collison
        if (!collison_info.has_collided)
            return false;

        real_T dt_real = static_cast<real_T>(dt);

        //are we going away from collison?
        real_T vnext_normal_mag = collison_info.normal.dot(next.twist.linear + next.accelerations.linear * dt_real);
        //if not then we need collison response
        if (vnext_normal_mag >= 0.0f)
            return false;

        //get current velocity's reflection
        Vector3r vcur_avg = current.twist.linear + current.accelerations.linear * dt_real;

        //if current velocity is going away from collison then don't reflect it
        if (-collison_info.normal.dot(vcur_avg) >= 0.0f) {
            /********** Core collison response ***********/

            //get average angular velocity
            Vector3r angular_avg = current.twist.angular + current.accelerations.angular * dt_real;

            //contact point vector
            Vector3r r = collison_info.impact_point - collison_info.position;

            //see if impact is straight at body's surface (assuming its box)
            Vector3r normal_body = VectorMath::transformToBodyFrame(collison_info.normal, current.pose.orientation);
            if (Utils::isApproximatelyEqual(std::abs(normal_body.x()), 1.0f, 0.05f) 
                || Utils::isApproximatelyEqual(std::abs(normal_body.y()), 1.0f, 0.05f)
                || Utils::isApproximatelyEqual(std::abs(normal_body.z()), 1.0f, 0.05f)) {

                //think of collison occured along the surface, not at point
                r = Vector3r::Zero();

            }

            //velocity at contact point
            Vector3r vcur_avg_body = VectorMath::transformToBodyFrame(vcur_avg, current.pose.orientation);
            Vector3r contact_vel_body = vcur_avg_body + angular_avg.cross(r);

            /*
                GafferOnGames - Collison response with columb friction
                http://gafferongames.com/virtual-go/collision-response-and-coulomb-friction/
                Assuming collison is with static fixed body,
                impulse magnitude = j = -(1 + R)V.N / (1/m + (I'(r X N) X r).N)
                Physics Part 3, Collison Response, Chris Hecker, eq 4(a)
                http://chrishecker.com/images/e/e7/Gdmphys3.pdf
                V(t+1) = V(t) + j*N / m
            */
            real_T impulse_mag_denom = 1.0f / body.getMass() + 
                (body.getInertiaInv() * r.cross(normal_body))
                .cross(r)
                .dot(normal_body);
            real_T impulse_mag = -contact_vel_body.dot(normal_body) * (1 + body.getRestitution()) / impulse_mag_denom;

            next.twist.linear = vcur_avg + collison_info.normal * (impulse_mag / body.getMass());
            next.twist.angular = angular_avg + r.cross(normal_body) * impulse_mag;

            //above would modify component in direction of normal
            //we will use friction to modify component in direction of tangent
            Vector3r contact_tang_body = contact_vel_body - normal_body * normal_body.dot(contact_vel_body);
            Vector3r contact_tang_unit_body = contact_tang_body.normalized();
            real_T friction_mag_denom =  1.0f / body.getMass() + 
                (body.getInertiaInv() * r.cross(contact_tang_unit_body))
                .cross(r)
                .dot(contact_tang_unit_body);
            real_T friction_mag = -contact_tang_body.norm() * body.getFriction() / friction_mag_denom;

            Vector3r contact_tang_unit = VectorMath::transformToWorldFrame(contact_tang_unit_body, current.pose.orientation);
            next.twist.linear += contact_tang_unit * friction_mag;
            next.twist.angular += r.cross(contact_tang_unit_body) * (friction_mag / body.getMass());
        }
        else //keep the current velocity
            next.twist.linear = vcur_avg;

        //there is no acceleration during collison response
        next.accelerations.linear = Vector3r::Zero();
        next.accelerations.angular = Vector3r::Zero();
                
        //do not use current.pose because it might be invalid
        next.pose.position = collison_info.position + (collison_info.normal * collison_info.penetration_depth) + next.twist.linear * (dt_real * kCollisionResponseCycles);
        next_wrench = Wrench::zero();

        return true;
    }

    bool getNextKinematicsOnGround(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        /************************* reset state if we have hit the ground ************************/
        real_T min_z_over_ground = body.getEnvironment().getState().min_z_over_ground;
        grounded_ = 0;
        if (min_z_over_ground <= next.pose.position.z()) {
            grounded_ = 1;
            next.pose.position.z() = min_z_over_ground;

            real_T z_proj = static_cast<real_T>(next.twist.linear.z() + next.accelerations.linear.z() * dt);
            if (Utils::isDefinitelyLessThan(0.0f, z_proj)) {
                grounded_ = 2;
                next.twist = Twist::zero();
                next.accelerations.linear = Vector3r::Zero();
                next.accelerations.angular = Vector3r::Zero();
                //reset roll/pitch - px4 seems to have issue with this
                real_T r, p, y;
                VectorMath::toEulerianAngle(current.pose.orientation, p, r, y);
                next.pose.orientation = VectorMath::toQuaternion(0, 0, y);

                next_wrench = Wrench::zero();
            }
        }

        return grounded_ != 0;
    }

    static Wrench getDragWrench(const PhysicsBody& body, const Quaternionr& orientation, 
        const Vector3r& linear_vel, const Vector3r& angular_vel_body)
    {
        //add linear drag due to velocity we had since last dt seconds
        //drag vector magnitude is proportional to v^2, direction opposite of velocity
        //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
        //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
        //http://physics.stackexchange.com/questions/304742/angular-drag-on-body
        //similarly calculate angular drag
        //note that angular velocity, acceleration, torque are already in body frame


        Wrench wrench = Wrench::zero();
        real_T air_density = body.getEnvironment().getState().air_density;

        for (uint vi = 0; vi < body.dragVertexCount(); ++vi) {
            const auto& vertex = body.getDragVertex(vi);
            Vector3r vel_vertex = VectorMath::transformToBodyFrame(linear_vel, orientation) + angular_vel_body.cross(vertex.getPosition());
            real_T vel_comp = vertex.getNormal().dot(vel_vertex);
            if (vel_comp > 0.1f) { //TODO: create const or param
                Vector3r drag_force = vertex.getNormal() * (- vertex.getDragFactor() * air_density * vel_comp * vel_comp);
                Vector3r drag_torque = vertex.getPosition().cross(drag_force);

                wrench.force += drag_force;
                wrench.torque += drag_torque;
            }
        }

        //convert force to world frame, leave torque to local frame
        wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

        return wrench;
    }

    static Wrench getBodyWrench(const PhysicsBody& body, const Quaternionr& orientation)
    {
        //set wrench sum to zero
        Wrench wrench = Wrench::zero();

        //calculate total force on rigid body's center of gravity
        for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
            //aggregate total
            const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
            const auto& vertex_wrench = vertex.getWrench();
            wrench += vertex_wrench;

            //add additional torque due to force applies farther than COG
            // tau = r X F
            wrench.torque +=  vertex.getPosition().cross(vertex_wrench.force);
        }

        //convert force to world frame, leave torque to local frame
        wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

        return wrench;
    }

    void getNextKinematicsNoCollison(TTimeDelta dt, const PhysicsBody& body, const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench)
    {
        real_T dt_real = static_cast<real_T>(dt);

        /************************* Get force and torque acting on body ************************/
        //set wrench sum to zero
        Wrench body_wrench = getBodyWrench(body, current.pose.orientation);

        //add linear drag due to velocity we had since last dt seconds
        //drag vector magnitude is proportional to v^2, direction opposite of velocity
        //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
        //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
        Vector3r avg_velocity = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
        Vector3r avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
        Wrench drag_wrench = getDragWrench(body, current.pose.orientation, avg_velocity, avg_angular);

        next_wrench = body_wrench + drag_wrench;


        /************************* Update accelerations due to force and torque ************************/
        //get new acceleration due to force - we'll use this acceleration in next time step
        next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;

        //get new angular acceleration
        //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
        //we will use torque to find out the angular acceleration
        //angular momentum L = I * omega
        Vector3r angular_momentum = body.getInertia() * avg_angular;
        Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
        //new angular acceleration - we'll use this acceleration in next time step
        next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;



        /************************* Update pose and twist after dt ************************/
        //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
        next.pose.position = current.pose.position + avg_velocity * dt_real;
        next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);

        //use angular velocty in body frame to calculate angular displacement in last dt seconds
        real_T angle_per_unit = avg_angular.norm();
        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
            //convert change in angle to unit quaternion
            AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
            Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
            /*
                Add change in angle to previous orientation.
                Proof that this is q0 * q1:
                If rotated vector is qx*v*qx' then qx is attitude
                Initially we have q0*v*q0'
                Lets transform this to body coordinates to get
                q0'*(q0*v*q0')*q0
                Then apply q1 rotation on it to get
                q1(q0'*(q0*v*q0')*q0)q1'
                Then transform back to world coordinate
                q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
                which simplifies to
                q0(q1(v)q1')q0'
                Thus new attitude is q0q1
            */
            next.pose.orientation = current.pose.orientation * angle_dt_q;
            if (VectorMath::hasNan(next.pose.orientation))
                Utils::DebugBreak();

            //re-normalize quaternion to avoid accumulating error
            next.pose.orientation.normalize();
        } 
        else //no change in angle, because angular velocity is zero (normalized vector is undefined)
            next.pose.orientation = current.pose.orientation;

        next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);
    }

private:
    std::stringstream debug_string_;
    int grounded_;
};

}} //namespace
#endif
