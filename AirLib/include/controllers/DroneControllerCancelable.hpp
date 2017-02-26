// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DroneControlServer_hpp
#define air_DroneControlServer_hpp

#include "common/Common.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "Waiter.hpp"
#include <atomic>

namespace msr { namespace airlib {

class DroneControllerCancelable : CancelableBase {
public:
    DroneControllerCancelable(DroneControllerBase* drone)
        :controller_(drone), is_cancelled_(false)
    {
        //auto vehicle_params = controller_->getVehicleParams();
        //auto fence = std::make_shared<CubeGeoFence>(VectorMath::Vector3f(-1E10, -1E10, -1E10), VectorMath::Vector3f(1E10, 1E10, 1E10), vehicle_params.distance_accuracy);
        //auto safety_eval = std::make_shared<SafetyEval>(vehicle_params, fence);
    }

    bool armDisarm(bool arm)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->armDisarm(arm, *this);
    }
    void setOffboardMode(bool is_set)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        controller_->setOffboardMode(is_set);
    }
    void setSimulationMode(bool is_set)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        controller_->setSimulationMode(is_set);
    }
    void setUserInputs(const vector<float>& inputs)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        controller_->setUserInputs(inputs);
    }
    void start()
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        controller_->start();
    }
    void stop()
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        controller_->stop();
    }
    bool takeoff(float max_wait_seconds)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->takeoff(max_wait_seconds, *this);
    }
    bool land()
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->land(*this);
    }
    bool goHome()
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->goHome(*this);
    }


    bool moveByAngle(float pitch, float roll, float z, float yaw, float duration)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveByAngle(pitch, roll, z, yaw, duration, *this);
    }

    bool moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode, *this);
    }

    bool moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode, *this);
    }

    bool moveOnPath(const vector<Vector3r>& path, float velocity, DrivetrainType drivetrain, const YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveOnPath(path, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead, *this);
    }

    bool moveToPosition(float x, float y, float z, float velocity, DrivetrainType drivetrain,
        const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveToPosition(x, y, z, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead, *this);
    }

    bool moveToZ(float z, float velocity, const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveToZ(z, velocity, yaw_mode, lookahead, adaptive_lookahead, *this);
    }

    bool moveByManual(float vx_max, float vy_max, float z_min, DrivetrainType drivetrain, const YawMode& yaw_mode, float duration)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->moveByManual(vx_max, vy_max, z_min, drivetrain, yaw_mode, duration, *this);
    }

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->setSafety(enable_reasons, obs_clearance, obs_startegy,
            obs_avoidance_vel, origin, xy_length, max_z, min_z);
    }

    bool rotateToYaw(float yaw, float margin)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->rotateToYaw(yaw, margin, *this);
    }

    bool rotateByYawRate(float yaw_rate, float duration)
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->rotateByYawRate(yaw_rate, duration, *this);
    }

    bool hover()
    {
        CallLock lock(action_mutex_, &is_cancelled_);
        return controller_->hover(*this);
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

    double timestampNow()
    {
        return controller_->timestampNow();
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
        return std::to_string(Utils::getTimeSinceEpochMillis());
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


    /*** Implementation of CancelableBase ***/
    bool isCancelled() override  
    {
        return is_cancelled_;
    }
    virtual void cancelAllTasks() override
    {
        CallLock lock(action_mutex_, &is_cancelled_);
    }
    /*** Implementation of CancelableBase ***/

private:// types
    struct CallLock {
        CallLock(std::mutex& mtx, std::atomic_bool* is_cancelled)
            : is_cancelled_(is_cancelled)
        {
            //tell other call to exit
            *is_cancelled_ = true;

            //wait to acquire lock
            lock_ = std::unique_lock<std::mutex>(mtx);

            //reset cancellation before we proceed
            *is_cancelled_ = false;
        }

        ~CallLock()
        {
            //reset cancellation because call is completed now
            *is_cancelled_ = false;

            //lock_ will be destroyed automatically and release any locks
        }

    private:
        std::unique_lock<std::mutex> lock_;
        std::atomic_bool* is_cancelled_;
    };

private: //vars
    DroneControllerBase* controller_;
    std::atomic_bool is_cancelled_;
    std::mutex action_mutex_;
};

}} //namespace
#endif
