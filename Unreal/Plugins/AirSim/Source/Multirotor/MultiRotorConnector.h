#pragma once

#include "CoreMinimal.h"
//TODO: all code except setRotorSpeed requires VehiclePawnBase.
//May be we should have MultiRotorPawnBase so we don't need FlyingPawn.h
#include "vehicles/multirotor/api/MultirotorApi.hpp"
#include "VehiclePawnWrapper.h"
#include "vehicles/multirotor/MultiRotor.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "controllers/VehicleConnectorBase.hpp"
#include "ManualPoseController.h"
#include <chrono>
#include "api/ControlServerBase.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include <future>


class MultiRotorConnector : public msr::airlib::VehicleConnectorBase
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::ControllerBase ControllerBase;
    typedef msr::airlib::MultiRotor MultiRotor;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;


public:
    virtual ~MultiRotorConnector();

    //VehicleConnectorBase interface
    //implements game interface to update pawn
    MultiRotorConnector(VehiclePawnWrapper* vehicle_paw_wrapper, msr::airlib::MultiRotorParams* vehicle_params, 
        bool enable_rpc, std::string api_server_address, uint16_t api_server_port,
        UManualPoseController* manual_pose_controller);
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    virtual void startApiServer() override;
    virtual void stopApiServer() override;
    virtual bool isApiServerStarted() override;
    virtual msr::airlib::VehicleControllerBase* getController() override;

    //PhysicsBody interface
    //this just wrapped around MultiRotor physics body
    virtual void reset() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual Pose getPose() override;
    virtual Kinematics::State getTrueKinematics() override;


    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override;
    virtual int getSegmentationObjectID(const std::string& mesh_name) override;

    virtual msr::airlib::ImageCaptureBase* getImageCapture() override;

    virtual void printLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0) override;
    virtual Pose getActorPose(const std::string& actor_name) override;
    virtual CameraInfo getCameraInfo(int camera_id) const override;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) override;

private:
    void detectUsbRc();
    const msr::airlib::RCData& getRCData();  
    void resetPrivate();

private:
    MultiRotor vehicle_;
    std::vector<std::string> controller_messages_;
    std::unique_ptr<msr::airlib::Environment> environment_;
    VehiclePawnWrapper* vehicle_pawn_wrapper_;

    msr::airlib::MultiRotorParams* vehicle_params_;
    std::unique_ptr<msr::airlib::MultirotorApi> controller_cancelable_;
    std::unique_ptr<msr::airlib::ControlServerBase> rpclib_server_;

    struct RotorInfo {
        real_T rotor_speed = 0;
        int rotor_direction = 0;
        real_T rotor_thrust = 0;
        real_T rotor_control_filtered = 0;
    };
    unsigned int rotor_count_;
    std::vector<RotorInfo> rotor_info_;

    CollisionResponseInfo collision_response_info;

    bool enable_rpc_;
    std::string api_server_address_;
    uint16_t api_server_port_;
    msr::airlib::DroneControllerBase* controller_;
    UManualPoseController* manual_pose_controller_;

    SimJoyStick joystick_;
    SimJoyStick::State joystick_state_;
    msr::airlib::RCData rc_data_;

    bool pending_pose_collisions_;
    enum class PendingPoseStatus {
        NonePending, RenderStatePending, RenderPending
    } pending_pose_status_;
    Pose pending_pose_; //force new pose through API

    //reset must happen while World is locked so its async task initiated from API thread
    bool reset_pending_;
    bool did_reset_;
    std::packaged_task<void()> reset_task_;

    Pose last_pose_; //for trace lines showing vehicle path
    Pose last_debug_pose_; //for purposes such as comparing recorded trajectory
};
