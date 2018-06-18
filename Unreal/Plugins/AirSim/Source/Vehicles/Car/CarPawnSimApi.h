#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "PawnEvents.h"
#include "PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "physics//Kinematics.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"

class CarPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;
    
public:
    virtual ~CarPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    CarPawnSimApi(ACarPawn* pawn, const NedTransform& global_transform, PawnEvents* pawn_events,
        const common_utils::UniqueValueMap<std::string, APIPCamera*>& cameras, UClass* pip_camera_class, UParticleSystem* collision_display_template,
        const CarPawnApi::CarControls&  keyboard_controls,
        UWheeledVehicleMovementComponent* movement, const msr::airlib::GeoPoint& home_geopoint);

    virtual void reset() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;

    virtual std::string getRecordFileLine(bool is_header_line) const override;

    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    msr::airlib::CarApiBase* getVehicleApi()
    {
        return vehicle_api_.get();
    }

private:
    void createVehicleApi(ACarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
    void updateCarControls();

private:
    std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
    std::vector<std::string> vehicle_api_messages_;

    //storing reference from pawn
    const CarPawnApi::CarControls& keyboard_controls_;

    CarPawnApi::CarControls joystick_controls_;
    CarPawnApi::CarControls current_controls_;
};
