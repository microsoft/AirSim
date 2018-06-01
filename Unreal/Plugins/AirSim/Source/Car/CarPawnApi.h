#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "WheeledVehicleMovementComponent4W.h"
#include "physics/Kinematics.hpp"


class CarPawnApi : public msr::airlib::CarApiBase {
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    CarPawnApi(UWheeledVehicleMovementComponent* movement, const msr::airlib::GeoPoint& home_geopoint);

    virtual void setCarControls(const CarApiBase::CarControls& controls) override;

    virtual CarApiBase::CarState getCarState() const override;

    virtual void reset() override;

    virtual msr::airlib::GeoPoint getHomeGeoPoint() const override;

    virtual void enableApiControl(bool is_enabled) override;
    virtual bool isApiControlEnabled() const override;
    virtual bool armDisarm(bool arm) override;

    virtual const CarApiBase::CarControls& getCarControls() const override;

    virtual ~CarPawnApi();

private:
    UWheeledVehicleMovementComponent* movement_;
    msr::airlib::GeoPoint  home_geopoint_;
    bool api_control_enabled_ = false;
    CarControls last_controls_;
};