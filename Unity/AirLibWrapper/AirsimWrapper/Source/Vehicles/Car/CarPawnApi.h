#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "physics/Kinematics.hpp"

class CarPawnApi
{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

public:
    CarPawnApi(const msr::airlib::Kinematics::State* pawn_kinematics,
               const std::string car_name, msr::airlib::CarApiBase* vehicle_api);

    void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);
    msr::airlib::CarApiBase::CarState getCarState() const;
    void enableApi(bool enable);

    void reset();
    void update();

    virtual ~CarPawnApi();

private:
    msr::airlib::CarApiBase::CarControls last_controls_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    std::string car_name_;
    msr::airlib::CarApiBase* vehicle_api_;
};
