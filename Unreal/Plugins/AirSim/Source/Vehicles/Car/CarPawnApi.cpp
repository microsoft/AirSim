#include "CarPawnApi.h"
#include "AirBlueprintLib.h"

#include "PhysXVehicleManager.h"

CarPawnApi::CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
                       msr::airlib::CarApiBase* vehicle_api)
    : pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
    movement_ = pawn->GetVehicleMovement();
}

void CarPawnApi::updateMovement(const msr::airlib::CarApiBase::CarControls& controls)
{
    last_controls_ = controls;

    if (!controls.is_manual_gear && movement_->GetTargetGear() < 0)
        movement_->SetTargetGear(0, true); //in auto gear we must have gear >= 0
    if (controls.is_manual_gear && movement_->GetTargetGear() != controls.manual_gear)
        movement_->SetTargetGear(controls.manual_gear, controls.gear_immediate);

    movement_->SetThrottleInput(controls.throttle);
    movement_->SetSteeringInput(controls.steering);
    movement_->SetBrakeInput(controls.brake);
    movement_->SetHandbrakeInput(controls.handbrake);
    movement_->SetUseAutoGears(!controls.is_manual_gear);
}

msr::airlib::CarApiBase::CarState CarPawnApi::getCarState() const
{
    msr::airlib::CarApiBase::CarState state(
        movement_->GetForwardSpeed() / 100, //cm/s -> m/s
        movement_->GetCurrentGear(),
        movement_->GetEngineRotationSpeed(),
        movement_->GetEngineMaxRotationSpeed(),
        last_controls_.handbrake,
        *pawn_kinematics_,
        vehicle_api_->clock()->nowNanos());
    return state;
}

void CarPawnApi::reset()
{
    vehicle_api_->reset();

    last_controls_ = msr::airlib::CarApiBase::CarControls();
    auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps) {
            phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
            phys_comp->SetSimulatePhysics(false);
        }
        movement_->ResetMoveState();
        movement_->SetActive(false);
        movement_->SetActive(true, true);
        vehicle_api_->setCarControls(msr::airlib::CarApiBase::CarControls());
        updateMovement(msr::airlib::CarApiBase::CarControls());

        auto pv = movement_->PVehicle;
        if (pv) {
            pv->mWheelsDynData.setToRestState();
        }
        auto pvd = movement_->PVehicleDrive;
        if (pvd) {
            pvd->mDriveDynData.setToRestState();
        }
    },
                                             true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps)
            phys_comp->SetSimulatePhysics(true);
    },
                                             true);
}

void CarPawnApi::update()
{
    vehicle_api_->updateCarState(getCarState());
    vehicle_api_->update();
}

CarPawnApi::~CarPawnApi() = default;
