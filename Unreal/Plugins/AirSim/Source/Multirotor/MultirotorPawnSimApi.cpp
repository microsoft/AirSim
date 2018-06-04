#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(APawn* pawn, const NedTransform& global_transform, CollisionSignal& collision_signal,
    const std::map<std::string, APIPCamera*>& cameras,
    UManualPoseController* manual_pose_controller, const GeoPoint& home_geopoint)
    : PawnSimApi(pawn, global_transform, collision_signal, cameras),
      manual_pose_controller_(manual_pose_controller)
{
    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);

    createVehicleApi();

    //setup physics vehicle
    phys_vehicle_ = std::unique_ptr<MultiRotor>(new MultiRotor(vehicle_params_.get(), vehicle_api_.get(), 
        getPose(), home_geopoint));
    rotor_count_ = phys_vehicle_->wrenchVertexCount();
    rotor_info_.assign(rotor_count_, RotorInfo());

    //initialize private vars
    last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
}

void MultirotorPawnSimApi::createVehicleApi()
{
    //create vehicle params
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
}

std::string MultirotorPawnSimApi::getLogLine() const
{
    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    //TODO: because this bug we are using alternative code with stringstream
    //https://answers.unrealengine.com/questions/664905/unreal-crashes-on-two-lines-of-extremely-simple-st.html

    std::string line;
    line.append(std::to_string(timestamp_millis)).append("\t")
        .append(std::to_string(kinematics->pose.position.x())).append("\t")
        .append(std::to_string(kinematics->pose.position.y())).append("\t")
        .append(std::to_string(kinematics->pose.position.z())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.w())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.x())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.y())).append("\t")
        .append(std::to_string(kinematics->pose.orientation.z())).append("\t");

    return line;

    //std::stringstream ss;
    //ss << timestamp_millis << "\t";
    //ss << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
    //ss << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
    //ss << "\n";
    //return ss.str();

}

const msr::airlib::Kinematics::State* MultirotorPawnSimApi::getGroundTruthKinematics() const
{
    return & phys_vehicle_->getKinematics();
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

    //update ground level
    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == getPawn()) {
        FVector delta_position;
        FRotator delta_rotation;

        manual_pose_controller_->updateDeltaPosition(dt);
        manual_pose_controller_->getDeltaPose(delta_position, delta_rotation);
        manual_pose_controller_->resetDelta();
        Vector3r delta_position_ned = getNedTransform().toLocalNed(delta_position);
        Quaternionr delta_rotation_ned = getNedTransform().toNed(delta_rotation.Quaternion());

        auto pose = phys_vehicle_->getPose();
        pose.position += delta_position_ned;
        pose.orientation = pose.orientation * delta_rotation_ned;
        pose.orientation.normalize();

        phys_vehicle_->setPose(pose);
    }

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending)
        phys_vehicle_->setPose(pending_phys_pose_);
        
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

    //update rotor animations
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        RotorInfo* info = &rotor_info_[i];
        static_cast<AFlyingPawn*>(getPawn())->
            setRotorSpeed(i, info->rotor_speed * info->rotor_direction);
    }

    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == getPawn()) {
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(getCollisionInfo().collision_count), LogDebugLevel::Failure);
    }
    else {
        //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(collision_response.collision_count_non_resting), LogDebugLevel::Failure);
    }

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    pending_phys_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::reset()
{
    PawnSimApi::reset();
    phys_vehicle_->reset();
}

void MultirotorPawnSimApi::update()
{
    PawnSimApi::update();

    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    phys_vehicle_->update();
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
    phys_vehicle_->reportState(reporter);
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    return phys_vehicle_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

