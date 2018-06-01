#include "MultirotorSimApi.h"
#include "FlyingPawn.h" 
#include "AirBlueprintLib.h"
#include <exception>

using namespace msr::airlib;

MultirotorSimApi::MultirotorSimApi(msr::airlib::MultirotorApiBase* vehicle_api, msr::airlib::MultiRotorParams* vehicle_params,
    UManualPoseController* manual_pose_controller)
    : vehicle_api_(vehicle_api), vehicle_params_(vehicle_params), manual_pose_controller_(manual_pose_controller)
{
    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);

    //setup physics vehicle
    phys_vehicle_ = std::unique_ptr<MultiRotor>(new MultiRotor(vehicle_params_, vehicle_api_, 
        getPose(), vehicle_api_->getHomeGeoPoint(), environment_));
    rotor_count_ = phys_vehicle_->wrenchVertexCount();
    rotor_info_.assign(rotor_count_, RotorInfo());

    //setup RC
    if (getRemoteControlID() >= 0)
        detectUsbRc();

    //initialize private vars
    last_pose_ = pending_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;

    std::string message;
    if (!vehicle_api_->isReady(message)) {
        UAirBlueprintLib::LogMessage(FString("Vehicle was not initialized: "), FString(message.c_str()), LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
    }
}

void MultirotorSimApi::detectUsbRc()
{
    joystick_.getJoyStickState(getRemoteControlID(), joystick_state_);

    rc_data_.is_initialized = joystick_state_.is_initialized;

    if (rc_data_.is_initialized)
        UAirBlueprintLib::LogMessageString("RC Controller on USB: ", joystick_state_.pid_vid, LogDebugLevel::Informational);
    else
        UAirBlueprintLib::LogMessageString("RC Controller on USB not detected: ", 
            std::to_string(joystick_state_.connection_error_code), LogDebugLevel::Informational);
}

msr::airlib::RCData MultirotorSimApi::getRCData() const
{
    joystick_.getJoyStickState(getRemoteControlID(), joystick_state_);

    rc_data_.is_valid = joystick_state_.is_valid;

    if (rc_data_.is_valid) {
        //-1 to 1 --> 0 to 1
        rc_data_.throttle = (joystick_state_.left_y + 1) / 2;

        //convert 0 to 1 -> -1 to 1
        rc_data_.yaw = joystick_state_.left_x; 
        rc_data_.roll = joystick_state_.right_x;
        rc_data_.pitch = - joystick_state_.right_y;

        //TODO: add fields for z axis?

        //last 8 bits are not used for now
        rc_data_.switch1 = joystick_state_.buttons & 0x0001 ? 1 : 0; //front-upper-left
        rc_data_.switch2 = joystick_state_.buttons & 0x0002 ? 1 : 0; //front-upper-right
        rc_data_.switch3 = joystick_state_.buttons & 0x0004 ? 1 : 0; //top-right-left
        rc_data_.switch4 = joystick_state_.buttons & 0x0008 ? 1 : 0; //top-right-left
        rc_data_.switch5 = joystick_state_.buttons & 0x0010 ? 1 : 0; //top-left-right
        rc_data_.switch6 = joystick_state_.buttons & 0x0020 ? 1 : 0; //top-right-right
        rc_data_.switch7 = joystick_state_.buttons & 0x0040 ? 1 : 0; //top-left-left
        rc_data_.switch8 = joystick_state_.buttons & 0x0080 ? 1 : 0; //top-right-left


        UAirBlueprintLib::LogMessageString("Joystick (T,R,P,Y,Buttons): ", Utils::stringf("%f, %f, %f %f, %d",
            rc_data_.throttle, rc_data_.roll, rc_data_.pitch, rc_data_.yaw, joystick_state_.buttons), LogDebugLevel::Informational);

        //TODO: should below be at controller level info?
        UAirBlueprintLib::LogMessageString("RC Mode: ", rc_data_.switch1 == 0 ? "Angle" : "Rate", LogDebugLevel::Informational);

        UAirBlueprintLib::LogMessage(FString("Joystick (Switches): "), FString::FromInt(joystick_state_.buttons) + ", " +
            FString::FromInt(rc_data_.switch1) + ", " + FString::FromInt(rc_data_.switch2) + ", " + FString::FromInt(rc_data_.switch3) + ", " + FString::FromInt(rc_data_.switch4)
            + ", " + FString::FromInt(rc_data_.switch5) + ", " + FString::FromInt(rc_data_.switch6) + ", " + FString::FromInt(rc_data_.switch7) + ", " + FString::FromInt(rc_data_.switch8),
            LogDebugLevel::Informational);
    }
    //else don't waste time

    return rc_data_;
}

void MultirotorSimApi::updateRenderedState(float dt)
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
        Vector3r delta_position_ned = getNedTransform().toNedMeters(delta_position, false);
        Quaternionr delta_rotation_ned = getNedTransform().toQuaternionr(delta_rotation.Quaternion(), true);

        auto pose = phys_vehicle_->getPose();
        pose.position += delta_position_ned;
        pose.orientation = pose.orientation * delta_rotation_ned;
        pose.orientation.normalize();

        phys_vehicle_->setPose(pose);
    }

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending)
        phys_vehicle_->setPose(pending_pose_);
        
    last_pose_ = phys_vehicle_->getPose();
    
    collision_response_info = phys_vehicle_->getCollisionResponseInfo();

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
}

void MultirotorSimApi::updateRendering(float dt)
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

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    if (!VectorMath::hasNan(last_pose_)) {
        if (pending_pose_status_ ==  PendingPoseStatus::RenderPending) {
            VehicleSimApi::setPose(last_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            VehicleSimApi::setPose(last_pose_, false);
    }

    //update rotor animations
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        RotorInfo* info = &rotor_info_[i];
        static_cast<AFlyingPawn*>(getPawn())->
            setRotorSpeed(i, info->rotor_speed * info->rotor_direction);
    }

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == getPawn()) {
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(getCollisionInfo().collision_count), LogDebugLevel::Failure);
    }
    else {
        //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response_info.collision_count_raw), LogDebugLevel::Unimportant);
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(collision_response_info.collision_count_non_resting), LogDebugLevel::Failure);
    }

    /************************************************           for debugging        *****************************************************/
    //Kinematics::State kinematics_estimated = vehicle_api_->getKinematicsEstimated();
    //Kinematics::State kinematics_true = phys_vehicle_->getKinematics();
    //UAirBlueprintLib::LogMessageString("Position (true): ", VectorMath::toString(kinematics_true.pose.position), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Position (est): ", VectorMath::toString(kinematics_estimated.pose.position), LogDebugLevel::Informational);

    //UAirBlueprintLib::LogMessageString("Lin Velocity (true): ", VectorMath::toString(kinematics_true.twist.linear), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Lin Velocity (est): ", VectorMath::toString(kinematics_estimated.twist.linear), LogDebugLevel::Informational);

    //UAirBlueprintLib::LogMessageString("Ang Velocity (true): ", VectorMath::toString(kinematics_true.twist.angular), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Ang Velocity (est): ", VectorMath::toString(kinematics_estimated.twist.angular), LogDebugLevel::Informational);

    //UAirBlueprintLib::LogMessageString("Lin Accel (true): ", VectorMath::toString(kinematics_true.accelerations.linear), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Lin Accel (est): ", VectorMath::toString(kinematics_estimated.accelerations.linear), LogDebugLevel::Informational);

    //UAirBlueprintLib::LogMessageString("Ang Accel (true): ", VectorMath::toString(kinematics_true.accelerations.angular), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Ang Accel (est): ", VectorMath::toString(kinematics_estimated.accelerations.angular), LogDebugLevel::Informational);

    //UAirBlueprintLib::LogMessageString("Orien (true): ", VectorMath::toString(kinematics_true.pose.orientation), LogDebugLevel::Informational);
    //UAirBlueprintLib::LogMessageString("Orien (est): ", VectorMath::toString(kinematics_estimated.pose.orientation), LogDebugLevel::Informational);
}

void MultirotorSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    pending_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorSimApi::reset()
{
    VehicleSimApi::reset();
    rc_data_ = RCData();
    phys_vehicle_->reset();
}

void MultirotorSimApi::update()
{
    VehicleSimApi::update();

    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    phys_vehicle_->update();
}

void MultirotorSimApi::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    FVector unrealPosition = getUUPosition();
    reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
    phys_vehicle_->reportState(reporter);
}

MultirotorSimApi::UpdatableObject* MultirotorSimApi::getPhysicsBody()
{
    return phys_vehicle_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

