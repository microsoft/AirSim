#pragma once

#include "vehicles/Warthog/api/WarthogApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "WarthogPawn.h"

class WarthogPawnApi
{
public:
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    WarthogPawnApi(AWarthogPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
               msr::airlib::WarthogApiBase* vehicle_api);

    void updateMovement(const msr::airlib::WarthogApiBase::WarthogControls& controls);

    msr::airlib::WarthogApiBase::WarthogState getWarthogState() const;

    void reset();
    void update();

    virtual ~WarthogPawnApi();

private:
    msr::airlib::WarthogApiBase::WarthogControls last_controls_;
    AWarthogPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::WarthogApiBase* vehicle_api_;
};
