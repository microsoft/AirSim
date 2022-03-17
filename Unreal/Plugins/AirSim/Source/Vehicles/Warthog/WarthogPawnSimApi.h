#pragma once

#include "CoreMinimal.h"

#include "WarthogPawn.h"
#include "WarthogPawnApi.h"
#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/warthog/api/WarthogApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

class WarthogPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;

public:
    virtual void initialize() override;
    virtual ~WarthogPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    WarthogPawnSimApi(const Params& params,
                  const msr::airlib::WarthogApiBase::WarthogControls& keyboard_controls);

    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;

    virtual std::string getRecordFileLine(bool is_header_line) const override;

    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    msr::airlib::WarthogApiBase* getVehicleApi() const
    {
        return vehicle_api_.get();
    }

    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }

protected:
    virtual void resetImplementation() override;

private:
    void updateWarthogControls();

private:
    std::unique_ptr<msr::airlib::WarthogApiBase> vehicle_api_;
    std::unique_ptr<WarthogPawnApi> pawn_api_;
    std::vector<std::string> vehicle_api_messages_;

    //storing reference from pawn
    const msr::airlib::WarthogApiBase::WarthogControls& keyboard_controls_;

    msr::airlib::WarthogApiBase::WarthogControls joystick_controls_;
    msr::airlib::WarthogApiBase::WarthogControls current_controls_;
};