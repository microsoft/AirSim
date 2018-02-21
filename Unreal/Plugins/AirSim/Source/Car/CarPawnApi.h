#pragma once


#include "vehicles/car/api/CarApiBase.hpp"
#include "VehiclePawnWrapper.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "physics/Kinematics.hpp"


class CarPawnApi : public msr::airlib::CarApiBase {
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    CarPawnApi(VehiclePawnWrapper* pawn, UWheeledVehicleMovementComponent* movement);



    virtual bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override;

    virtual void simPrintLogMessage(const std::string& message, const std::string& message_param = "", unsigned char severity = 0) override;

    virtual int simGetSegmentationObjectID(const std::string& mesh_name) override;

    virtual msr::airlib::CollisionInfo getCollisionInfo() override;

    virtual std::vector<uint8_t> simGetImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) override;
    virtual std::vector<ImageCaptureBase::ImageResponse> simGetImages(
        const std::vector<ImageCaptureBase::ImageRequest>& requests) override;

    virtual void setCarControls(const CarApiBase::CarControls& controls) override;

    virtual CarApiBase::CarState getCarState() override;

    virtual void reset() override;

    virtual void simSetPose(const msr::airlib::Pose& pose, bool ignore_collision) override;

    virtual msr::airlib::Pose simGetPose() override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;

    virtual const CarApiBase::CarControls& getCarControls() const override;

    virtual msr::airlib::Pose simGetObjectPose(const std::string& actor_name) override;
    virtual msr::airlib::CameraInfo getCameraInfo(int camera_id) const override;
    virtual void setCameraOrientation(int camera_id, const msr::airlib::Quaternionr& orientation) override;

    virtual ~CarPawnApi();

private:
    VehiclePawnWrapper* pawn_;
    UWheeledVehicleMovementComponent* movement_;
    bool api_control_enabled_ = false;
    CarControls last_controls_;
};