// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_RealMultirotorConnector_hpp
#define air_RealMultirotorConnector_hpp

#include "controllers/VehicleConnectorBase.hpp"

namespace msr { namespace airlib {

class RealMultirotorConnector : public VehicleConnectorBase
{
public:
    RealMultirotorConnector(VehicleControllerBase* controller)
        : controller_(controller)
    {
    }

    virtual void updateRenderedState(float dt) override
    {
        unused(dt);
    }

    virtual void updateRendering(float dt) override
    {
        unused(dt);
    }

    virtual void startApiServer() override
    {
    }

    virtual void stopApiServer() override
    {
    }

    virtual bool isApiServerStarted() override
    {
        return false;
    }

    virtual VehicleControllerBase* getController() override
    {
        return controller_;
    }

    virtual ImageCaptureBase* getImageCapture() override
    {
        //TODO: we need to support this but with only scene image type
        throw std::logic_error("getImageCapture() call is only supported for simulation");
    }

    virtual void setPose(const Pose& pose, bool ignore_collision) override
    {
        unused(pose);
        unused(ignore_collision);
        throw std::logic_error("setPose() call is only supported for simulation");
    }

    virtual Pose getPose() override
    {
        throw std::logic_error("getPose() call is only supported for simulation");
    }

    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override
    {
        unused(mesh_name);
        unused(object_id);
        unused(is_name_regex);
        throw std::logic_error("setSegmentationObjectID() call is only supported for simulation");
    }
    virtual int getSegmentationObjectID(const std::string& mesh_name) override
    {
        unused(mesh_name);
        throw std::logic_error("getSegmentationObjectID() call is only supported for simulation");
    }

    virtual Kinematics::State getTrueKinematics() override
    {
        throw std::logic_error("getTrueKinematics() call is only supported for simulation");
    }

    virtual void printLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0)  override
    {
        unused(message);
        unused(message_param);
        unused(severity);
    }

    virtual Pose getActorPose(const std::string& actor_name) override
    {
        unused(actor_name);
        return msr::airlib::Pose();
    }

    virtual CameraInfo getCameraInfo(int camera_id) const override
    {
        unused(camera_id);
        throw std::logic_error("getCameraInfo() call is not implemented for this vehicle");
    }

    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) override
    {
        unused(camera_id);
        unused(orientation);

        throw std::logic_error("setCameraOrientation() call is not implemented for this vehicle");
    }

private:
    VehicleControllerBase* controller_;
};


}} //namespace
#endif