#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "CarPawn.h"

class CarPawnApi
{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

public:
    CarPawnApi(CarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, 
               const std::string car_name, msr::airlib::CarApiBase* vehicle_api);

    void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);
    msr::airlib::CarApiBase::CarState getCarState() const;

    void reset();
    void update();

    virtual ~CarPawnApi();

private:
    msr::airlib::CarApiBase::CarControls last_controls_;
    CarPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    std::string car_name_;
    msr::airlib::CarApiBase* vehicle_api_;
};
