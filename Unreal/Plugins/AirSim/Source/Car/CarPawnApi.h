#pragma once


#include "vehicles/car/api/CarApiBase.hpp"
#include "VehiclePawnWrapper.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "physics/Kinematics.hpp"


class CarPawnApi : public msr::airlib::CarApiBase {
public:
    typedef msr::airlib::VehicleCameraBase VehicleCameraBase;

    CarPawnApi(VehiclePawnWrapper* pawn, UWheeledVehicleMovementComponent* movement);

    virtual std::vector<VehicleCameraBase::ImageResponse> simGetImages(
        const std::vector<VehicleCameraBase::ImageRequest>& request) override;

    virtual bool simSetSegmentationObjectID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false) override;

    virtual void simPrintLogMessage(const std::string& message, std::string message_param = "", unsigned char severity = 0) override;

    virtual int simGetSegmentationObjectID(const std::string& mesh_name) override;

    virtual msr::airlib::CollisionInfo getCollisionInfo() override;

    virtual std::vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type) override;

    virtual void setCarControls(const CarApiBase::CarControls& controls) override;

    virtual CarApiBase::CarState getCarState() override;

    virtual void reset() override;

    virtual void simSetPose(const msr::airlib::Pose& pose, bool ignore_collision) override;

    virtual msr::airlib::Pose simGetPose() override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;

    virtual const CarApiBase::CarControls& getCarControls() const;


    virtual ~CarPawnApi();

private:
    VehiclePawnWrapper* pawn_;
    UWheeledVehicleMovementComponent* movement_;
    bool api_control_enabled_ = false;
    CarControls last_controls_;
};