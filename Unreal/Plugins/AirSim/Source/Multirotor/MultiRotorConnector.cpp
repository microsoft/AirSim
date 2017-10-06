#include "MultiRotorConnector.h"

#ifdef AIRLIB_NO_RPC
#include "api/DebugApiServer.hpp"
#else

#if defined _WIN32 || defined _WIN64
#include "AllowWindowsPlatformTypes.h"
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#if defined _WIN32 || defined _WIN64
#include "HideWindowsPlatformTypes.h"
#endif

#endif

#include "FlyingPawn.h" 
#include "AirBlueprintLib.h"
#include "NedTransform.h"
#include <exception>

using namespace msr::airlib;

MultiRotorConnector::MultiRotorConnector(VehiclePawnWrapper* vehicle_pawn_wrapper, 
    msr::airlib::MultiRotorParams* vehicle_params, bool enable_rpc, 
    std::string api_server_address, uint16_t api_server_port,
    UManualPoseController* manual_pose_controller)
{
    enable_rpc_ = enable_rpc;
    api_server_address_ = api_server_address;
    api_server_port_ = api_server_port;
    vehicle_pawn_wrapper_ = vehicle_pawn_wrapper;
    manual_pose_controller_ = manual_pose_controller;

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = vehicle_pawn_wrapper->getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    vehicle_pawn_wrapper->setPose(pose, false);

    vehicle_params_ = vehicle_params;

    vehicle_.initialize(vehicle_params_, vehicle_pawn_wrapper_->getPose(), 
        vehicle_pawn_wrapper_->getHomePoint(), environment_);

    controller_ = static_cast<msr::airlib::DroneControllerBase*>(vehicle_.getController());

    for (int camera_index = 0; camera_index < vehicle_pawn_wrapper_->getCameraCount(); ++camera_index) {
        controller_->simAddCamera(vehicle_pawn_wrapper_->getCameraConnector(camera_index));
    }

    if (controller_->getRemoteControlID() >= 0)
        detectUsbRc();

    rotor_count_ = vehicle_.wrenchVertexCount();
    rotor_info_.assign(rotor_count_, RotorInfo());

    last_pose = Pose::nanPose();
    last_debug_pose = Pose::nanPose();

    std::string message;
    if (!vehicle_.getController()->isAvailable(message)) {
        UAirBlueprintLib::LogMessage(FString("Vehicle was not initialized: "), FString(message.c_str()), LogDebugLevel::Failure);
        UAirBlueprintLib::LogMessage("Tip: check connection info in settings.json", "", LogDebugLevel::Informational);
    }
}

msr::airlib::VehicleCameraBase* MultiRotorConnector::getCamera(unsigned int index)
{
    return vehicle_pawn_wrapper_->getCameraConnector(index);
}

MultiRotorConnector::~MultiRotorConnector()
{
    stopApiServer();
}

msr::airlib::VehicleControllerBase* MultiRotorConnector::getController()
{
    return controller_;
}


void MultiRotorConnector::detectUsbRc()
{
    joystick_.getJoyStickState(controller_->getRemoteControlID(), joystick_state_);

    rc_data_.is_initialized = joystick_state_.is_initialized;

    if (rc_data_.is_initialized)
        UAirBlueprintLib::LogMessage(TEXT("RC Controller on USB: "), "Detected", LogDebugLevel::Informational);
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

void MultiRotorConnector::updateRenderedState()
{
    //Utils::log("------Render tick-------");

    //move collison info from rendering engine to vehicle
    const CollisionInfo& collision_info = vehicle_pawn_wrapper_->getCollisonInfo();
    vehicle_.setCollisionInfo(collision_info);

    //update ground level
    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == vehicle_pawn_wrapper_->getPawn()) {
        FVector delta_position;
        FRotator delta_rotation;

        manual_pose_controller_->getActorDeltaPose(delta_position, delta_rotation, true);
        Vector3r delta_position_ned = NedTransform::toNedMeters(delta_position, false);
        Quaternionr delta_rotation_ned = NedTransform::toQuaternionr(delta_rotation.Quaternion(), true);

        auto pose = vehicle_.getPose();
        pose.position += delta_position_ned;
        pose.orientation = pose.orientation * delta_rotation_ned;
        pose.orientation.normalize();

        vehicle_.setPose(pose);
    }

    last_pose = vehicle_.getPose();
    
    collision_response_info = vehicle_.getCollisionResponseInfo();
    last_debug_pose = controller_->getDebugPose();

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
    //this must be the first call so controller can change the state before anything else we do
    controller_->simNotifyRender();

    try {
        controller_->reportTelemetry(dt);
    }
    catch (std::exception &e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    if (!VectorMath::hasNan(last_pose.position)) {
        vehicle_pawn_wrapper_->setPose(last_pose, false);
        vehicle_pawn_wrapper_->setDebugPose(last_debug_pose);
    }

    //update rotor animations
    for (unsigned int i = 0; i < rotor_count_; ++i) {
        RotorInfo* info = &rotor_info_[i];
        static_cast<AFlyingPawn*>(vehicle_pawn_wrapper_->getPawn())->
            setRotorSpeed(i, info->rotor_speed * info->rotor_direction);
    }

    for (auto i = 0; i < controller_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(controller_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    if (manual_pose_controller_ != nullptr && manual_pose_controller_->getActor() == vehicle_pawn_wrapper_->getPawn()) {
        UAirBlueprintLib::LogMessage(TEXT("Collison Count:"), FString::FromInt(vehicle_pawn_wrapper_->getCollisonInfo().collison_count), LogDebugLevel::Failure);
    }
    else {
        //UAirBlueprintLib::LogMessage(TEXT("Collison (raw) Count:"), FString::FromInt(collision_response_info.collison_count_raw), LogDebugLevel::Unimportant);
        UAirBlueprintLib::LogMessage(TEXT("Collison Count:"), FString::FromInt(collision_response_info.collison_count_non_resting), LogDebugLevel::Failure);
    }
}


void MultiRotorConnector::startApiServer()
{
    if (enable_rpc_) {
        controller_cancelable_.reset(new msr::airlib::DroneControllerCancelable(
            vehicle_.getController()));

#ifdef AIRLIB_NO_RPC
    rpclib_server_.reset(new msr::airlib::DebugApiServer());
#else
    rpclib_server_.reset(new msr::airlib::MultirotorRpcLibServer(
        controller_cancelable_.get(), api_server_address_, api_server_port_));
#endif

        rpclib_server_->start();
        UAirBlueprintLib::LogMessageString("API server started at ", 
            api_server_address_ == "" ? "(default)" : api_server_address_.c_str(), LogDebugLevel::Informational);
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);

}
void MultiRotorConnector::stopApiServer()
{
    if (rpclib_server_ != nullptr) {
        controller_cancelable_->cancelAllTasks();
        rpclib_server_->stop();
        rpclib_server_.reset(nullptr);
        controller_cancelable_.reset(nullptr);
    }
}

bool MultiRotorConnector::isApiServerStarted()
{
    return rpclib_server_ != nullptr;
}

//*** Start: UpdatableState implementation ***//
void MultiRotorConnector::reset()
{
    VehicleConnectorBase::reset();

    //TODO: should this be done in MultiRotor.hpp
    //controller_->reset();

    rc_data_ = RCData();
    vehicle_pawn_wrapper_->reset();    //we do flier resetPose so that flier is placed back without collisons
    vehicle_.reset();
}

void MultiRotorConnector::update()
{
    VehicleConnectorBase::update();

    //this is high frequency physics tick, flier gets ticked at rendering frame rate
    vehicle_.update();
}

void MultiRotorConnector::reportState(StateReporter& reporter)
{
    // report actual location in unreal coordinates so we can plug that into the UE editor to move the drone.
    if (vehicle_pawn_wrapper_ != nullptr) {
        FVector unrealPosition = vehicle_pawn_wrapper_->getPosition();
        reporter.writeValue("unreal pos", NedTransform::toVector3r(unrealPosition, 1.0f, false));
        vehicle_.reportState(reporter);
    }
}

MultiRotorConnector::UpdatableObject* MultiRotorConnector::getPhysicsBody()
{
    return vehicle_.getPhysicsBody();
}
//*** End: UpdatableState implementation ***//

