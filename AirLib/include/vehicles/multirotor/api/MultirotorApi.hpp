// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlServer_hpp
#define air_DroneControlServer_hpp

#include "common/Common.hpp"
#include "common/common_utils/WorkerThread.hpp"
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "controllers/VehicleConnectorBase.hpp"
#include "api/VehicleApiBase.hpp"
#include "common/Waiter.hpp"
#include <atomic>
#include <thread>
#include <memory>

using namespace msr::airlib;

namespace msr { namespace airlib {

// We want to make it possible for MultirotorRpcLibClient to call the offboard movement methods (moveByAngle, moveByVelocity, etc) at a high
// rate, like 30 times a second.  But we also want these movement methods to drive the drone at a reliable rate which we do inside
// DroneControllerBase using the Waiter object so it pumps the virtual commandVelocity method at a fixed rate defined by getCommandPeriod.
// This fixed rate is needed by the drone flight controller (for example PX4) because the flight controller usually reverts to
// a failsafe operation like hover if it stop receiving these offboard control messages at that rate (for safety reasons).
// So moveByVelocity takes a duration, and DroneControllerBase pumps commandVelocity at the getCommandPeriod for that duration.
// How ever this would block the server RPC thread until that duration is complete, which would stop the MultirotorRpcLibClient from being
// able to send a new velocity at high rate.  So we have to decouple the client from the server side moveByVelocity control loop.
// And we do that decoupling with a thread which we call the "offboard_control" thread.  Then each client side call is passed to
// this thread so that the RPC call does NOT block, and the client can proceed with the next command.  But we also have to guarentee
// here that only one operation is allowed at a time and that is what the CallLock object is for.  It cancels previous operation then
// sets up the new operation.

class MultirotorApi : public VehicleApiBase {
public:
    MultirotorApi(VehicleConnectorBase* vehicle)
        : vehicle_(vehicle)
    {
        controller_ = static_cast<DroneControllerBase*>(vehicle->getController());

        //auto vehicle_params = controller_->getVehicleParams();
        //auto fence = std::make_shared<CubeGeoFence>(VectorMath::Vector3f(-1E10, -1E10, -1E10), VectorMath::Vector3f(1E10, 1E10, 1E10), vehicle_params.distance_accuracy);
        //auto safety_eval = std::make_shared<SafetyEval>(vehicle_params, fence);
    }
    virtual ~MultirotorApi() = default;

    bool armDisarm(bool arm)
    {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        return controller_->armDisarm(arm, *pending_);
    }

    void setSimulationMode(bool is_set)
    {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        controller_->setSimulationMode(is_set);
    }

    bool takeoff(float max_wait_seconds)
    {
        // takeoff might use waitForZ which is a loop, but since this method is given max_wait_seconds argument
        // client can decide not to wait by passing zero.
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        return controller_->takeoff(max_wait_seconds, *pending_);
    }
    bool land(float max_wait_seconds)
    {
        // takeoff might use a loop waiting for landed state, but since this method is given max_wait_seconds argument
        // client can decide not to wait by passing zero.
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        return controller_->land(max_wait_seconds, *pending_);
    }

    bool goHome()
    {
        // it is assumed this is not using offboard control loop, it is triggering an onboard "RTL" feature in the drone itself.
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        return controller_->goHome(*pending_);
    }

    bool moveByAngle(float pitch, float roll, float z, float yaw, float duration)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveByAngle>(controller_, pitch, roll, z, yaw, duration);
        return enqueueCommand(cmd);
    }

    bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveByVelocity>(controller_, vx, vy, vz, duration, drivetrain, yaw_mode);
        return enqueueCommand(cmd);
    }

    bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveByVelocityZ>(controller_, vx, vy, z, duration, drivetrain, yaw_mode);
        return enqueueCommand(cmd);
    }

    bool moveOnPath(const vector<Vector3r>& path, float velocity, float max_wait_seconds, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveOnPath>(controller_, path, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
        return enqueueCommandAndWait(cmd, max_wait_seconds);
    }

    bool moveToPosition(float x, float y, float z, float velocity, float max_wait_seconds, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveToPosition>(controller_, x, y, z, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
        return enqueueCommandAndWait(cmd, max_wait_seconds);
    }

    bool moveToZ(float z, float velocity, float max_wait_seconds, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveToZ>(controller_, z, velocity, yaw_mode, lookahead, adaptive_lookahead);
        return enqueueCommandAndWait(cmd, max_wait_seconds);
    }

    bool moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<MoveByManual>(controller_, vx_max, vy_max, z_min, duration, drivetrain, yaw_mode);
        return enqueueCommand(cmd);
    }

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
    {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        return controller_->setSafety(enable_reasons, obs_clearance, obs_startegy,
            obs_avoidance_vel, origin, xy_length, max_z, min_z);
    }

    bool rotateToYaw(float yaw, float max_wait_seconds, float margin)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<RotateToYaw>(controller_, yaw, margin);
        return enqueueCommandAndWait(cmd, max_wait_seconds);
    }


    bool rotateByYawRate(float yaw_rate, float duration)
    {
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<RotateByYawRate>(controller_, yaw_rate, duration);
        return enqueueCommand(cmd);
    }

    bool hover()
    {
        // hover is implemented using moveToZ which is also an offboard control loop.
        std::shared_ptr<OffboardCommand> cmd = std::make_shared<Hover>(controller_);
        return enqueueCommand(cmd);
    }

    /************************* State APIs *********************************/
    MultirotorState getMultirotorState()
    {
        MultirotorState state;
        state.kinematics_estimated = controller_->getKinematicsEstimated();
        state.collision = controller_->getCollisionInfo();
        state.kinematics_true = vehicle_->getTrueKinematics();
        state.gps_location = controller_->getGpsLocation();
        state.timestamp = controller_->clock()->nowNanos();

        return state;
    }

    Vector3r getPosition()
    {
        return controller_->getPosition();
    }

    Vector3r getVelocity()
    {
        return controller_->getVelocity();
    }

    Quaternionr getOrientation()
    {
        return controller_->getOrientation();
    }

    DroneControllerBase::LandedState getLandedState()
    {
        return controller_->getLandedState();
    }

    virtual CollisionInfo getCollisionInfo() override
    {
        return controller_->getCollisionInfo();
    }

    RCData getRCData()
    {
        return controller_->getRCData();
    }


    //TODO: add GPS health, accuracy in API
    GeoPoint getGpsLocation()
    {
        return controller_->getGpsLocation();
    }

    bool isSimulationMode()
    {
        return controller_->isSimulationMode();
    }

    void getStatusMessages(std::vector<std::string>& messages)
    {
        controller_->getStatusMessages(messages);
    }

    virtual void cancelAllTasks()
    {
        offboard_thread_.cancel();
    }


    /******************* VehicleApiBase implementtaion ********************/
    virtual GeoPoint getHomeGeoPoint() override
    {
        return controller_->getHomeGeoPoint();
    }
    virtual void enableApiControl(bool is_enabled) override
    {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_);
        pending_ = std::make_shared<DirectCancelableBase>();
        controller_->enableApiControl(is_enabled);
    }

    virtual vector<ImageCaptureBase::ImageResponse> simGetImages(const vector<ImageCaptureBase::ImageRequest>& requests) override
    {
        vector<ImageCaptureBase::ImageResponse> responses;
        ImageCaptureBase* image_capture = vehicle_->getImageCapture();
        image_capture->getImages(requests, responses);
        return responses;
    }
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) override
    {
        vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(camera_id, image_type)};
        const vector<ImageCaptureBase::ImageResponse>& response = simGetImages(request);
        if (response.size() > 0)
            return response.at(0).image_data_uint8;
        else 
            return vector<uint8_t>();
    }

    virtual void simPrintLogMessage(const std::string& message, const std::string& message_param, unsigned char severity) override
    {
        vehicle_->printLogMessage(message, message_param, severity);
    }

    virtual Pose simGetObjectPose(const std::string& actor_name) override
    {
        return vehicle_->getActorPose(actor_name);
    }

    virtual void simSetPose(const Pose& pose, bool ignore_collision) override
    {
        vehicle_->setPose(pose, ignore_collision);
    }
    virtual Pose simGetPose() override
    {
        return vehicle_->getPose();
    }

    virtual bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override
    {
        return vehicle_->setSegmentationObjectID(mesh_name, object_id, is_name_regex);
    }

    virtual int simGetSegmentationObjectID(const std::string& mesh_name) override
    {
        return vehicle_->getSegmentationObjectID(mesh_name);
    }

    virtual CameraInfo getCameraInfo(int camera_id) const override
    {
        return vehicle_->getCameraInfo(camera_id);
    }

    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) override
    {
        vehicle_->setCameraOrientation(camera_id, orientation);
    }

    virtual bool isApiControlEnabled() const override
    {
        return controller_->isApiControlEnabled();
    }

    virtual void reset() override
    {
        vehicle_->reset();
    }

    virtual void setRCData(const RCData& data)
    {
        controller_->setRCData(data);
    }


    /*** Implementation of CancelableBase ***/

private:// types
    // Trivial CancelableBase used for the synchronous commands (non-offboard control)
    // These commands do not use the WorkerThread, they are executed synchronously, but
    // they may still be cancelable.
    class DirectCancelableBase : public CancelableBase
    {
    public:
        virtual void execute() override {};
    };
    struct CallLock {
        CallLock(DroneControllerBase* controller, std::mutex& mtx, std::mutex& cancel_mutex, std::shared_ptr<CancelableBase> pending, bool is_loop_command = false)
            : controller_(controller)
        {
            //tell other call to exit (being careful to protect the interleaving of is_cancelled_ = true/is_cancelled_ = false).
            std::unique_lock<std::mutex> cancelLock(cancel_mutex);
            if (pending != nullptr) {
                pending->cancel();
            }

            //wait to acquire lock
            lock_ = std::unique_lock<std::mutex>(mtx);
            
            if (is_loop_command) {
                if (!controller_->loopCommandPre())
                    throw VehicleControllerException("Cannot start the command because loopCommandPre returned failed status");
                else
                    loop_post_needed = true;
            }
        }

        ~CallLock()
        {
            if (loop_post_needed)
                controller_->loopCommandPost();

            //lock_ will be destroyed automatically and release any locks
        }

    private:
        std::unique_lock<std::mutex> lock_;
        bool loop_post_needed = false;
        DroneControllerBase* controller_;
    };

    class OffboardCommand : public CancelableBase {
        DroneControllerBase* controller_;
    public:
        OffboardCommand(DroneControllerBase* controller) {
            controller_ = controller;
        }
        virtual void execute() override {
            executeImpl(controller_, *this);
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) = 0;
    };

    bool enqueueCommand(std::shared_ptr<OffboardCommand>& command) {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_, true);
        pending_ = command;
        offboard_thread_.enqueue(command);
        return true;
    }

    bool enqueueCommandAndWait(std::shared_ptr<OffboardCommand>& command, float max_wait_seconds) {
        CallLock lock(controller_, action_mutex_, cancel_mutex_, pending_, true);
        pending_ = command;
        if (max_wait_seconds > 0) {
            return offboard_thread_.enqueueAndWait(command, max_wait_seconds);
        }
        else {
            offboard_thread_.enqueue(command);
        }
        return true;
    }

    class MoveByAngle : public OffboardCommand {
        float pitch_, roll_, z_, yaw_, duration_;
    public:
        MoveByAngle(DroneControllerBase* controller, float pitch, float roll, float z, float yaw, float duration) : OffboardCommand(controller) {
            this->pitch_ = pitch;
            this->roll_ = roll;
            this->z_ = z;
            this->yaw_ = yaw;
            this->duration_ = duration;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveByAngle(pitch_, roll_, z_, yaw_, duration_, cancelable);
        }
    };

    class MoveByVelocity : public OffboardCommand {
        float vx_, vy_, vz_, duration_;
        YawMode yaw_mode_;
        DrivetrainType drivetrain_;
    public:
        MoveByVelocity(DroneControllerBase* controller, float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode) : OffboardCommand(controller) {
            this->vx_ = vx;
            this->vy_ = vy;
            this->vz_ = vz;
            this->duration_ = duration;
            this->drivetrain_ = drivetrain;
            this->yaw_mode_ = yaw_mode;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveByVelocity(vx_, vy_, vz_, duration_, drivetrain_, yaw_mode_, cancelable);
        }
    };

    class MoveByVelocityZ : public OffboardCommand {
        float vx_, vy_, z_, duration_;
        YawMode yaw_mode_;
        DrivetrainType drivetrain_;
    public:
        MoveByVelocityZ(DroneControllerBase* controller, float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode) : OffboardCommand(controller) {
            this->vx_ = vx;
            this->vy_ = vy;
            this->z_ = z;
            this->duration_ = duration;
            this->drivetrain_ = drivetrain;
            this->yaw_mode_ = yaw_mode;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveByVelocityZ(vx_, vy_, z_, duration_, drivetrain_, yaw_mode_, cancelable);
        }
    };

    class MoveOnPath : public OffboardCommand {
        vector<Vector3r> path_;
        float velocity_;
        DrivetrainType drivetrain_;
        YawMode yaw_mode_;
        float lookahead_;
        float adaptive_lookahead_;
    public:
        MoveOnPath(DroneControllerBase* controller, const vector<Vector3r>& path, float velocity, DrivetrainType drivetrain, const YawMode& yaw_mode,
            float lookahead, float adaptive_lookahead) : OffboardCommand(controller) {
            this->path_ = path;
            this->velocity_ = velocity;
            this->drivetrain_ = drivetrain;
            this->yaw_mode_ = yaw_mode;
            this->lookahead_ = lookahead;
            this->adaptive_lookahead_ = adaptive_lookahead;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveOnPath(path_, velocity_, drivetrain_, yaw_mode_, lookahead_, adaptive_lookahead_, cancelable);
        }
    };


    class MoveToPosition : public OffboardCommand {
        float x_;
        float y_;
        float z_;
        float velocity_;
        DrivetrainType drivetrain_;
        YawMode yaw_mode_;
        float lookahead_;
        float adaptive_lookahead_;
    public:
        MoveToPosition(DroneControllerBase* controller, float x, float y, float z, float velocity, DrivetrainType drivetrain,
            const YawMode& yaw_mode, float lookahead, float adaptive_lookahead) : OffboardCommand(controller) {
            this->x_ = x;
            this->y_ = y;
            this->z_ = z;
            this->velocity_ = velocity;
            this->drivetrain_ = drivetrain;
            this->yaw_mode_ = yaw_mode;
            this->lookahead_ = lookahead;
            this->adaptive_lookahead_ = adaptive_lookahead;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveToPosition(x_, y_, z_, velocity_, drivetrain_, yaw_mode_, lookahead_, adaptive_lookahead_, cancelable);
        }
    };


    class MoveToZ : public OffboardCommand {
        float z_;
        float velocity_;
        YawMode yaw_mode_;
        float lookahead_;
        float adaptive_lookahead_;
    public:
        MoveToZ(DroneControllerBase* controller, float z, float velocity, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead) : OffboardCommand(controller) {
            this->z_ = z;
            this->velocity_ = velocity;
            this->yaw_mode_ = yaw_mode;
            this->lookahead_ = lookahead;
            this->adaptive_lookahead_ = adaptive_lookahead;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveToZ(z_, velocity_, yaw_mode_, lookahead_, adaptive_lookahead_, cancelable);
        }
    };


    class MoveByManual : public OffboardCommand {
        float vx_max_;
        float vy_max_;
        float z_min_;
        YawMode yaw_mode_;
        DrivetrainType drivetrain_;
        float duration_;
    public:
        MoveByManual(DroneControllerBase* controller, float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode) : OffboardCommand(controller) {
            this->vx_max_ = vx_max;
            this->vy_max_ = vy_max;
            this->z_min_ = z_min;
            this->yaw_mode_ = yaw_mode;
            this->drivetrain_ = drivetrain;
            this->duration_ = duration;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->moveByManual(vx_max_, vy_max_, z_min_, duration_, drivetrain_, yaw_mode_, cancelable);
        }
    };


    class RotateToYaw : public OffboardCommand {
        float yaw_;
        float margin_;
    public:
        RotateToYaw(DroneControllerBase* controller, float yaw, float margin) : OffboardCommand(controller) {
            this->yaw_ = yaw;
            this->margin_ = margin;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->rotateToYaw(yaw_, margin_, cancelable);
        }
    };

    class RotateByYawRate : public OffboardCommand {
        float yaw_rate_;
        float duration_;
    public:
        RotateByYawRate(DroneControllerBase* controller, float yaw_rate, float duration) : OffboardCommand(controller) {
            this->yaw_rate_ = yaw_rate;
            this->duration_ = duration;
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->rotateByYawRate(yaw_rate_, duration_, cancelable);
        }
    };

    class Hover : public OffboardCommand {
    public:
        Hover(DroneControllerBase* controller) : OffboardCommand(controller) {
        }
        virtual void executeImpl(DroneControllerBase* controller, CancelableBase& cancelable) override {
            controller->hover(cancelable);
        }
    };

private: //vars
    VehicleConnectorBase* vehicle_ = nullptr;
    DroneControllerBase* controller_ = nullptr;
    WorkerThread offboard_thread_;
    std::mutex action_mutex_;
    std::mutex cancel_mutex_;
    std::shared_ptr<CancelableBase> pending_;
};

}} //namespace
#endif
