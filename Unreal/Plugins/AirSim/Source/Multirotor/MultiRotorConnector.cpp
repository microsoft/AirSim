#include "MultiRotorConnector.h"
#include "FlyingPawn.h" 
#include "AirBlueprintLib.h"
#include <exception>

using namespace msr::airlib;

MultiRotorConnector::MultiRotorConnector(VehiclePawnWrapper* wrapper, 
    msr::airlib::MultiRotorParams* vehicle_params,
    UManualPoseController* manual_pose_controller)
{
    wrapper_ = wrapper;
    manual_pose_controller_ = manual_pose_controller;

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = wrapper_->getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    wrapper_->setPose(pose, false);

    vehicle_params_ = vehicle_params;

    vehicle_.initialize(vehicle_params_, wrapper_->getPose(), 
        wrapper_->getHomePoint(), environment_);

    controller_ = static_cast<msr::airlib::MultirotorApiBase*>(vehicle_.getVehicleApi());

    if (controller_->getRemoteControlID() >= 0)
        detectUsbRc();

    rotor_count_ = vehicle_.wrenchVertexCount();
    rotor_info_.assign(rotor_count_, RotorInfo());

    last_pose_ = pending_pose_ = last_debug_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;

    wrapper_->setApi(std::unique_ptr<msr::airlib::VehicleApiBase>(
        new MultirotorApiBase(this)));

    std::string message;
    if (!vehicle_.getApi()->isAvailable(message)) {
        UAirBlueprintLib::LogMessage(FString("Vehicle was not initialized: "), FString(message.c_str()), LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
    }
}

msr::airlib::ImageCaptureBase* MultiRotorConnector::getImageCapture()
{
    return wrapper_->getImageCapture();
}

Kinematics::State MultiRotorConnector::getGroundTruthKinematics()
{
    return * wrapper_->getGroundTruthKinematics();
}

MultiRotorConnector::~MultiRotorConnector()
{
}

msr::airlib::VehicleApiBase* MultiRotorConnector::getVehicleApi()
{
    return controller_;
}

void MultiRotorConnector::detectUsbRc()
{
    joystick_.getJoyStickState(controller_->getRemoteControlID(), joystick_state_);

    rc_data_.is_initialized = joystick_state_.is_initialized;

    if (rc_data_.is_initialized)
        UAirBlueprintLib::LogMessageString("RC Controller on USB: ", joystick_state_.pid_vid, LogDebugLevel::Informational);
    else
        UAirBlueprintLib::LogMessageString("RC Controller on USB not detected: ", 
            std::to_string(joystick_state_.connection_error_code), LogDebugLevel::Informational);
}

const msr::airlib::RCData& MultiRotorConnector::getRCData()
{
    joystick_.getJoyStickState(controller_->getRemoteControlID(), joystick_state_);

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

void MultiRotorConnector::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = wrapper_->getCollisionInfo();
    vehicle_.setCollisionInfo(collision_info);

    //update ground level
    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == wrapper_->getPawn()) {
        FVector delta_position;
        FRotator delta_rotation;

        manual_pose_controller_->updateDeltaPosition(dt);
        manual_pose_controller_->getDeltaPose(delta_position, delta_rotation);
        manual_pose_controller_->resetDelta();
        Vector3r delta_position_ned = wrapper_->getNedTransform().toNedMeters(delta_position, false);
        Quaternionr delta_rotation_ned = wrapper_->getNedTransform().toQuaternionr(delta_rotation.Quaternion(), true);

        auto pose = vehicle_.getPose();
        pose.position += delta_position_ned;
        pose.orientation = pose.orientation * delta_rotation_ned;
        pose.orientation.normalize();

        vehicle_.setPose(pose);
    }

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending)
        vehicle_.setPose(pending_pose_);
        
    last_pose_ = vehicle_.getPose();
    
    collision_response_info = vehicle_.getCollisionResponseInfo();
    last_debug_pose_ = controller_->getDebugPose();

    //update rotor poses
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        const auto& rotor_output = vehicle_.getRotorOutput(i);
        RotorInfo* info = &rotor_info_[i];
        info->rotor_speed = rotor_output.speed;
        info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
        info->rotor_thrust = rotor_output.thrust;
        info->rotor_control_filtered = rotor_output.control_signal_filtered;
    }

    controller_->getStatusMessages(controller_messages_);

    if (controller_->getRemoteControlID() >= 0)
        controller_->setRCData(getRCData());
}

void MultiRotorConnector::updateRendering(float dt)
{
    //if we did reset then don't worry about synchrnozing states for this tick
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
        controller_->reportTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    if (!VectorMath::hasNan(last_pose_)) {
        if (pending_pose_status_ ==  PendingPoseStatus::RenderPending) {
            wrapper_->setPose(last_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            wrapper_->setPose(last_pose_, false);

        wrapper_->setDebugPose(last_debug_pose_);
    }

    //update rotor animations
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        RotorInfo* info = &rotor_info_[i];
        static_cast<AFlyingPawn*>(wrapper_->getPawn())->
            setRotorSpeed(i, info->rotor_speed * info->rotor_direction);
    }

    for (auto i = 0; i < controller_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == wrapper_->getPawn()) {
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(wrapper_->getCollisionInfo().collision_count), LogDebugLevel::Failure);
    }
    else {
        //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response_info.collision_count_raw), LogDebugLevel::Unimportant);
        UAirBlueprintLib::LogMessage(TEXT("Collision Count:"), FString::FromInt(collision_response_info.collision_count_non_resting), LogDebugLevel::Failure);
    }

    /************************************************           for debugging        *****************************************************/
    //Kinematics::State kinematics_estimated = controller_->getKinematicsEstimated();
    //Kinematics::State kinematics_true = vehicle_.getKinematics();
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

void MultiRotorConnector::setPose(const Pose& pose, bool ignore_collision)
{
    pending_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

Pose MultiRotorConnector::getPose()
{
    return vehicle_.getPose();
}

Pose MultiRotorConnector::getActorPose(const std::string& actor_name)
{
    msr::airlib::Pose pose;

    UAirBlueprintLib::RunCommandOnGameThread([&pose, &actor_name, this]() {
        pose = wrapper_->getActorPose(actor_name);
    }, true);

    return pose;
}

bool MultiRotorConnector::setSegmentationObjectID(const std::string& mesh_name, int object_id,
    bool is_name_regex)
{
    bool success;
    UAirBlueprintLib::RunCommandOnGameThread([mesh_name, object_id, is_name_regex, &success]() {
        success = UAirBlueprintLib::SetMeshStencilID(mesh_name, object_id, is_name_regex);
    }, true);
    return success;
}

void MultiRotorConnector::printLogMessage(const std::string& message, std::string message_param, unsigned char severity)
{
    wrapper_->printLogMessage(message, message_param, severity);
}

int MultiRotorConnector::getSegmentationObjectID(const std::string& mesh_name)
{
    return UAirBlueprintLib::GetMeshStencilID(mesh_name);
}

CameraInfo MultiRotorConnector::getCameraInfo(int camera_id) const
{
    return wrapper_->getCameraInfo(camera_id);
}

void MultiRotorConnector::setCameraOrientation(int camera_id, const Quaternionr& orientation) 
{
    UAirBlueprintLib::RunCommandOnGameThread([&camera_id, &orientation, this]() {
        wrapper_->setCameraOrientation(camera_id, orientation);
    }, true);
}

msr::airlib::MultirotorApiBase* MultiRotorConnector::getApi() const
{
    return static_cast<msr::airlib::MultirotorApiBase*>(wrapper_->getApi());
}


//*** Start: UpdatableState implementation ***//
void MultiRotorConnector::reset()
{
    resetPrivate();
}

void MultiRotorConnector::resetPrivate()
{
    VehicleSimBridgeBase::reset();

    //TODO: should this be done in MultiRotor.hpp?
    //controller_->reset();

    rc_data_ = RCData();
    wrapper_->reset();    //we do flier resetPose so that flier is placed back without collisions
    vehicle_.reset();
}

void MultiRotorConnector::update()
{
    VehicleSimBridgeBase::update();

    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    vehicle_.update();
}

void MultiRotorConnector::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    if (wrapper_ != nullptr) {
        FVector unrealPosition = wrapper_->getUUPosition();
        reporter.writeValue("unreal pos", Vector3r(unrealPosition.X, unrealPosition.Y, unrealPosition.Z));
        vehicle_.reportState(reporter);
    }
}

MultiRotorConnector::UpdatableObject* MultiRotorConnector::getPhysicsBody()
{
    return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

