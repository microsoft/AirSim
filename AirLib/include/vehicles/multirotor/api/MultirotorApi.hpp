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

    /************************* State APIs *********************************/
    MultirotorState getMultirotorState() const
    {
        MultirotorState state;
        state.kinematics_estimated = controller_->getKinematicsEstimated();
        state.collision = controller_->getCollisionInfo();
        state.kinematics_true = vehicle_->getTrueKinematics();
        //TODO: add GPS health, accuracy in API
        state.gps_location = getGpsLocation();
        state.timestamp = controller_->clock()->nowNanos();
        state.landed_state = controller_->getLandedState();
        state.rc_data = controller_->getRCData();
        controller_->getStatusMessages(state.controller_messages);

        return state;
    }

    virtual GeoPoint getGpsLocation() const = 0;

    /******************* VehicleApiBase implementation ********************/
    virtual void cancelPendingTasks() override
    {
        token_.cancel();
    }

    //redefine from base
    virtual void enableApiControl(bool is_enabled) override = 0;

    virtual CollisionInfo getCollisionInfo() const override
    {
        return controller_->getCollisionInfo();
    }




    virtual vector<ImageCaptureBase::ImageResponse> simGetImages(const vector<ImageCaptureBase::ImageRequest>& requests) const override
    {
        vector<ImageCaptureBase::ImageResponse> responses;
        ImageCaptureBase* image_capture = vehicle_->getImageCapture();
        image_capture->getImages(requests, responses);
        return responses;
    }
    virtual vector<uint8_t> simGetImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) const override
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

    virtual Pose simGetObjectPose(const std::string& actor_name) const override
    {
        return vehicle_->getActorPose(actor_name);
    }

    virtual void simSetPose(const Pose& pose, bool ignore_collision) override
    {
        vehicle_->setPose(pose, ignore_collision);
    }
    virtual Pose simGetPose() const override
    {
        return vehicle_->getPose();
    }

    virtual bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override
    {
        return vehicle_->setSegmentationObjectID(mesh_name, object_id, is_name_regex);
    }

    virtual int simGetSegmentationObjectID(const std::string& mesh_name) const override
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
    class CancelToken {
    public:
        CancelToken()
            : is_cancelled_(false)
        {
        }

        void reset()
        {
            is_cancelled_ = false;
        }

        bool isCancelled() const
        {
            return is_cancelled_;
        }

        void cancel()
        {
            is_cancelled_ = true;
        }

        bool sleep(double secs)
        {
            //We can pass duration directly to sleep_for however it is known that on 
            //some systems, sleep_for makes system call anyway even if passed duration 
            //is <= 0. This can cause 50us of delay due to context switch.
            if (isCancelled()) {
                return false;
            }

            TTimePoint start = ClockFactory::get()->nowNanos();
            static constexpr std::chrono::duration<double> MinSleepDuration(0);

            while (secs > 0 && !isCancelled() &&
                ClockFactory::get()->elapsedSince(start) < secs) {

                std::this_thread::sleep_for(MinSleepDuration);
            }

            return !isCancelled();
        }

        std::mutex& getWaitMutex()
        {
            return wait_mutex_;
        }

    private:
        std::atomic<bool> is_cancelled_;
        std::mutex wait_mutex_;
    };

    class SingleCall {
    public:
        SingleCall(CancelToken& token) 
            : token_(token) {

            if (!token_.getWaitMutex().try_lock()) {
                token_.cancel();
                token_.getWaitMutex().lock();
            }
            token_.reset();
        }

        ~SingleCall()
        {
            token_.reset();
            token_.getWaitMutex().unlock();
        }

    private:
        CancelToken& token_;
    };

private: //vars
    VehicleConnectorBase* vehicle_ = nullptr;
    DroneControllerBase* controller_ = nullptr;
    CancelToken token_;
};

}} //namespace
#endif
