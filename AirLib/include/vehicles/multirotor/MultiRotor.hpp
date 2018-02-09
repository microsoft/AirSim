// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_multirotor_hpp
#define msr_airlib_multirotor_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "Rotor.hpp"
#include "controllers/ControllerBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"


namespace msr { namespace airlib {

class MultiRotor : public PhysicsBody {
public:
    MultiRotor()
    {
        //allow default constructor with later call for initialize
    }
    MultiRotor(MultiRotorParams* params, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        initialize(params, initial_kinematic_state, environment);
    }
    void initialize(MultiRotorParams* params, const Pose& initial_pose, const GeoPoint& home_point, 
        std::unique_ptr<Environment>& environment)
    {
        //init physics vehicle
        auto initial_kinematics = Kinematics::State::zero();
        initial_kinematics.pose = initial_pose;
        Environment::State initial_environment;
        initial_environment.position = initial_kinematics.pose.position;
        initial_environment.geo_point = home_point;
        environment.reset(new Environment(initial_environment));
        
        initialize(params, initial_kinematics, environment.get());
    }
    void initialize(MultiRotorParams* params, const Kinematics::State& initial_kinematic_state, Environment* environment)
    {
        params_ = params;

        PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, initial_kinematic_state, environment);

        createRotors(*params_, rotors_, environment);
        createDragVertices();

        initSensors(*params_, getKinematics(), getEnvironment());

        getController()->setGroundTruth(this);
    }

    DroneControllerBase* getController()
    {
        return params_->getController();
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        //reset rotors, kinematics and environment
        PhysicsBody::reset();

        //reset inputs
        if (getController())
            getController()->reset();

        //reset sensors last after their ground truth has been reset
        resetSensors();
    }

    virtual void update() override
    {
        //update forces and environment as a result of last dt
        PhysicsBody::update();

        //Note that controller gets updated after kinematics gets updated in kinematicsUpdated
    }
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        PhysicsBody::reportState(reporter);

        reportSensors(*params_, reporter);

        //report rotors
        for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
            reporter.startHeading("", 1);
            reporter.writeValue("Rotor", rotor_index);
            reporter.endHeading(false, 1);
            rotors_.at(rotor_index).reportState(reporter);
        }
    }
    //*** End: UpdatableState implementation ***//


    //implement abstract methods from PhysicsBody
    virtual void kinematicsUpdated() override
    {
        updateSensors(*params_, getKinematics(), getEnvironment());

        getController()->update();

        //transfer new input values from controller to rotors
        for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
            rotors_.at(rotor_index).setControlSignal(
                getController()->getVertexControlSignal(rotor_index));
        }
    }

    //sensor getter
    const SensorCollection& getSensors() const
    {
        return params_->getSensors();
    }

    //physics body interface
    virtual uint wrenchVertexCount() const  override
    {
        return params_->getParams().rotor_count;
    }
    virtual PhysicsBodyVertex& getWrenchVertex(uint index)  override
    {
        return rotors_.at(index);
    }
    virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
    {
        return rotors_.at(index);
    }

    virtual uint dragVertexCount() const override
    {
        return static_cast<uint>(drag_vertices_.size());
    }
    virtual PhysicsBodyVertex& getDragVertex(uint index)  override
    {
        return drag_vertices_.at(index);
    }
    virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
    {
        return drag_vertices_.at(index);
    }

    virtual real_T getRestitution() const override
    {
        return params_->getParams().restitution;
    }
    virtual real_T getFriction()  const override
    {
        return params_->getParams().friction;
    }

    Rotor::Output getRotorOutput(uint rotor_index) const
    {
        return rotors_.at(rotor_index).getOutput();
    }

    virtual void setCollisionInfo(const CollisionInfo& collision_info) override
    {
        PhysicsBody::setCollisionInfo(collision_info);
        getController()->setCollisionInfo(collision_info);
    }

    virtual ~MultiRotor() = default;

private: //methods
    static void createRotors(const MultiRotorParams& params, vector<Rotor>& rotors, const Environment* environment)
    {
        rotors.clear();
        //for each rotor pose
        for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
            const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
            rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
        }
    }

    void reportSensors(MultiRotorParams& params, StateReporter& reporter)
    {
        params.getSensors().reportState(reporter);
    }

    void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
    {
        unused(state);
        unused(environment);
        params.getSensors().update();
    }

    void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
    {
        params.getSensors().initialize(&state, &environment);
    }

    void resetSensors()
    {
        params_->getSensors().reset();
    }

    void createDragVertices()
    {
        const auto& params = params_->getParams();

        //Drone is seen as central body that is connected to propellers via arm. We approximate central body as box of size x, y, z.
        //The drag depends on area exposed so we also add area of propellers to approximate drag they may introduce due to their area.
        //while moving along any axis, we find area that will be exposed in that direction
        real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
        real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;

        real_T top_bottom_area = params.body_box.x() * params.body_box.y();
        real_T left_right_area = params.body_box.x() * params.body_box.z();
        real_T front_back_area = params.body_box.y() * params.body_box.z();
        Vector3r drag_factor_unit = Vector3r(
            front_back_area + rotors_.size() * propeller_xsection, 
            left_right_area + rotors_.size() * propeller_xsection, 
            top_bottom_area + rotors_.size() * propeller_area) 
            * params.linear_drag_coefficient / 2; 

        //add six drag vertices representing 6 sides
        drag_vertices_.clear();
        drag_vertices_.emplace_back(Vector3r(0, 0, -params.body_box.z()), Vector3r(0, 0, -1), drag_factor_unit.z());
        drag_vertices_.emplace_back(Vector3r(0, 0,  params.body_box.z()), Vector3r(0, 0,  1), drag_factor_unit.z());
        drag_vertices_.emplace_back(Vector3r(0, -params.body_box.y(), 0), Vector3r(0, -1, 0), drag_factor_unit.y());
        drag_vertices_.emplace_back(Vector3r(0,  params.body_box.y(), 0), Vector3r(0,  1, 0), drag_factor_unit.y());
        drag_vertices_.emplace_back(Vector3r(-params.body_box.x(), 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
        drag_vertices_.emplace_back(Vector3r( params.body_box.x(), 0, 0), Vector3r( 1, 0, 0), drag_factor_unit.x());

    }

private: //fields
    MultiRotorParams* params_;

    //let us be the owner of rotors object
    vector<Rotor> rotors_;
    vector<PhysicsBodyVertex> drag_vertices_;
};

}} //namespace
#endif
