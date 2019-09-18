#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params)
    : PawnSimApi(params),
      pawn_events_(static_cast<MultirotorPawnEvents*>(params.pawn_events))
{
    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
    //setup physics vehicle
    phys_vehicle_ = std::unique_ptr<MultiRotor>(new MultiRotor(vehicle_params_.get(), vehicle_api_.get(),
        getKinematics(), getEnvironment()));
    rotor_count_ = phys_vehicle_->wrenchVertexCount();
    rotor_info_.assign(rotor_count_, RotorInfo());

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
}

void MultirotorPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    phys_vehicle_->setCollisionInfo(collision_info);

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending) {
        phys_vehicle_->setPose(pending_phys_pose_);
        pending_pose_status_ = PendingPoseStatus::RenderPending;
    }
        
    last_phys_pose_ = phys_vehicle_->getPose();
    
    collision_response = phys_vehicle_->getCollisionResponseInfo();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = phys_vehicle_->getRotorOutput(i);
        RotorInfo* info = &rotor_info_[i];
        info->rotor_speed = rotor_output.speed;
        info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
        info->rotor_thrust = rotor_output.thrust;
        info->rotor_control_filtered = rotor_output.control_signal_filtered;
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ ==  PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), 
        FString::FromInt(collision_response.collision_count_non_resting), LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getActuatorSignal().emit(rotor_info_);
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    pending_phys_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    phys_vehicle_->reset();
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    //update forces on vertices
    phys_vehicle_->update();

    //update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    phys_vehicle_->reportState(reporter);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    return phys_vehicle_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

