// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlServer_hpp
#define air_DroneControlServer_hpp

#include "common/Common.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "Waiter.hpp"
#include <atomic>

namespace msr { namespace airlib {

class DroneControllerCancelable  {
public:
    DroneControllerCancelable(DroneControllerBase* drone)
        :controller_(drone)
    {
        //auto vehicle_params = controller_->getVehicleParams();
        //auto fence = std::make_shared<CubeGeoFence>(VectorMath::Vector3f(-1E10, -1E10, -1E10), VectorMath::Vector3f(1E10, 1E10, 1E10), vehicle_params.distance_accuracy);
        //auto safety_eval = std::make_shared<SafetyEval>(vehicle_params, fence);
    }

    bool armDisarm(bool arm)
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->armDisarm(arm, cancelable_action_);
    }
    void setOffboardMode(bool is_set)
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        controller_->setOffboardMode(is_set);
    }
    void setSimulationMode(bool is_set)
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        controller_->setSimulationMode(is_set);
    }
    void start()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        controller_->start();
    }
    void stop()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        controller_->stop();
    }
    bool takeoff(float max_wait_seconds)
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->takeoff(max_wait_seconds, cancelable_action_);
    }
    bool land()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->land(cancelable_action_);
    }
    bool goHome()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->goHome(cancelable_action_);
    }


    bool moveByAngle(float pitch, float roll, float z, float yaw, float duration)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveByAngle(pitch, roll, z, yaw, duration, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveByAngle threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [&] {
            bool rc = false;
            try {
                rc = controller_->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveByVelocity threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveByVelocityZ threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool moveOnPath(const vector<Vector3r>& path, float velocity, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        // since we are async, we need a stable copy of the path
        current_path_ = path;
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveOnPath(current_path_, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveOnPath threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });

        return true;
    }

    bool moveToPosition(float x, float y, float z, float velocity, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveToPosition(x, y, z, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveToPosition threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool moveToZ(float z, float velocity, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveToZ(z, velocity, yaw_mode, lookahead, adaptive_lookahead, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveToZ threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool moveByManual(float vx_max, float vy_max, float z_min, DrivetrainType drivetrain, const YawMode& yaw_mode, float duration)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->moveByManual(vx_max, vy_max, z_min, drivetrain, yaw_mode, duration, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("moveByManual threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->setSafety(enable_reasons, obs_clearance, obs_startegy,
            obs_avoidance_vel, origin, xy_length, max_z, min_z);
    }

    bool rotateToYaw(float yaw, float margin)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);

        // run loop command in background task.
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->rotateToYaw(yaw, margin, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("rotateToYaw threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool rotateByYawRate(float yaw_rate, float duration)
    {
        CallLock lock(controller_, current_task_, cancelable_action_, true);
        current_task_ = std::async(std::launch::async, [=] {
            bool rc = false;
            try {
                rc = controller_->rotateByYawRate(yaw_rate, duration, cancelable_action_);
            }
            catch (std::exception& e) {
                // async exceptions have no way to get back to droneshell, so swallow them and unlock.
                Utils::logMessage("rotateByYawRate threw exception: %s", e.what());
            }
            cancelable_action_.complete();
            return rc;
        });
        return true;
    }

    bool hover()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
        return controller_->hover(cancelable_action_);
    }

    //status getters
    //TODO: add single call to get all of the state
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

    RCData getRCData()
    {
        return controller_->getRCData();
    }

    TTimePoint timestampNow()
    {
        return controller_->clock()->nowNanos();
    }
    GeoPoint getHomePoint()
    {
        return controller_->getHomePoint();
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

    bool isOffboardMode()
    {
        return controller_->isOffboardMode();
    }

    std::string getServerDebugInfo()
    {
        //for now this method just allows to see if server was started
        return std::to_string(Utils::getUnixTimeStamp());
    }

    void getStatusMessages(std::vector<std::string>& messages)
    {
        controller_->getStatusMessages(messages);
    }


    //request image
    void setImageTypeForCamera(int camera_id, DroneControllerBase::ImageType type)
    {
        controller_->setImageTypeForCamera(camera_id, type);
    }
    DroneControllerBase::ImageType getImageTypeForCamera(int camera_id)
    {
        return controller_->getImageTypeForCamera(camera_id);
    }
    //get/set image
    void setImageForCamera(int camera_id, DroneControllerBase::ImageType type, const vector<uint8_t>& image)
    {
        return controller_->setImageForCamera(camera_id, type, image);
    }
    vector<uint8_t> getImageForCamera(int camera_id, DroneControllerBase::ImageType type)
    {
        return controller_->getImageForCamera(camera_id, type);
    }

    bool isCancelled()  
    {
        return cancelable_action_.isCancelled();
    }
    virtual void cancelAllTasks()
    {
        CallLock lock(controller_, current_task_, cancelable_action_);
    }

private:// types

    // this implementation of CancelableBase ensures only one long running task at a time
    // using a mutex to lock entry/exit from those tasks.
    class CancelableAction : public CancelableBase {
    public:
        CancelableAction() : CancelableBase(&cancelled_) {
        }

        virtual void complete() override {
            action_mutex_.unlock();
            reset();
        }

        void enter() {
            //tell other call to exit
            cancel();

            // and wait for it to signal completion
            int retries = 50;
            while (!action_mutex_.try_lock() && retries > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
                retries--;
                if (retries == 0) {
                    throw VehicleControllerException("Previous long running command is not responding to cancel");
                }
            }

            reset();
        }

        void cancel() {
            cancelled_ = true;
        }
        void reset() {
            cancelled_ = false;
        }
    private:
        std::mutex action_mutex_; 
        std::atomic_bool cancelled_;
    };


    struct CallLock {

        CallLock(DroneControllerBase* controller, std::future<bool>& current_task, CancelableAction& cancelable_action, bool long_running = false)
            : controller_(controller), cancelable_action_(cancelable_action)
        {
            // if this is a long running task we need to wait for previous task to complete.
            cancelable_action_.enter();

            // this ensures we serialize any long running background tasks.
            if (current_task.valid()) {
                current_task.wait();
            }

            if (long_running) {
                if (!controller_->loopCommandPre()) {
                    cancelable_action_.complete();
                    throw VehicleControllerException("Cannot start the command because loopCommandPre returned failed status");
                }
                else
                    loop_post_needed = true;
            }
        }

        ~CallLock()
        {
            if (loop_post_needed)
                controller_->loopCommandPost();
            else
                cancelable_action_.complete();
        }

    private:
        bool loop_post_needed = false;
        DroneControllerBase* controller_;
        CancelableAction& cancelable_action_;
    };

private: //vars
    CancelableAction cancelable_action_;
    DroneControllerBase* controller_;
    std::future<bool> current_task_;
    vector<Vector3r> current_path_;
};

}} //namespace
#endif
