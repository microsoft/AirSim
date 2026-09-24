#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#if ENGINE_MAJOR_VERSION >= 5
#include "ChaosVehicleMovementComponent.h"
#else
#include "WheeledVehicleMovementComponent4W.h"
#endif
#include "physics/Kinematics.hpp"
#include "CarPawn.h"

class CarPawnApi
{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
               msr::airlib::CarApiBase* vehicle_api);

    void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);

    msr::airlib::CarApiBase::CarState getCarState() const;

    void reset();
    void update();

    virtual ~CarPawnApi();

private:
#if ENGINE_MAJOR_VERSION >= 5
    UChaosVehicleMovementComponent* movement_;
#else
    UWheeledVehicleMovementComponent* movement_;
#endif
    msr::airlib::CarApiBase::CarControls last_controls_;
    ACarPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::CarApiBase* vehicle_api_;
};
