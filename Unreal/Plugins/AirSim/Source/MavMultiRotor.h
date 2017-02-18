#pragma once

#include "control/MavLinkHelper.h"
#include "vehicles/controllers/MotorDirectController.hpp"
#include "vehicles/configs/Px4QuadX.hpp"
#include "vehicles/MultiRotor.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "control/DroneControlBase.hpp"
#include "VehicleBase.h"
#include "FlyingPawn.h"


class MavMultiRotor : public VehicleBase
{
public:
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::CollisionInfo CollisionInfo;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::ControllerBase ControllerBase;
    typedef msr::airlib::Px4QuadX Px4QuadX;
    typedef msr::airlib::Kinematics Kinematics;
    typedef msr::airlib::MultiRotor MultiRotor;
    typedef msr::airlib::Twist Twist;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
        
public:
    virtual ~MavMultiRotor() = default;

    //VehicleBase interface
    //implements game interface to update pawn
    void initialize(AFlyingPawn* vehicle_pawn);
    virtual void beginPlay() override;
    virtual void endPlay() override;
    virtual void updateRenderedState() override;
    virtual void updateRendering() override;

    //PhysicsBody interface
    //this just wrapped around MultiRotor physics body
    virtual void reset() override;
    virtual void update(real_T dt) override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    //provides way to control the drone
    msr::airlib::DroneControlBase* createOrGetDroneControl();

private:
    msr::airlib::MavLinkHelper::HILConnectionInfo getConnectionInfo();
    void openConnection();
    void closeConnection();

private:
    MultiRotor vehicle_;
    msr::airlib::MavLinkHelper mav_;
    std::vector<std::string> mav_messages_;
    msr::airlib::Environment environment_;
    AFlyingPawn* vehicle_pawn_;

    real_T rotor_speeds_[4];
    int rotor_directions_[4];
    real_T rotor_thrusts_[4];
    real_T rotor_controls_filtered_[4];
};
