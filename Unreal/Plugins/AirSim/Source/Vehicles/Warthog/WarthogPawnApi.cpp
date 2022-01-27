#include "WarthogPawnApi.h"
#include "AirBluePrintLib.h"

WarthogPawnApi::WarthogPawnApi(AWarthogPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
                               msr::airlib::WarthogApiBase* vehicle_api)
    : pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
}

void WarthogPawnApi::updateMovement(const msr::airlib::WarthogApiBase::WarthogControls& controls)
{
    last_controls_ = controls;

    //if (!controls.is_manual_gear && movement_->GetTargetGear() < 0)
    //   movement_->SetTargetGear(0, true); //in auto gear we must have gear >= 0
    //if (controls.is_manual_gear && movement_->GetTargetGear() != controls.manual_gear)
    //   movement_->SetTargetGear(controls.manual_gear, controls.gear_immediate);
    pawn_->SetLinearVelocity(controls.linear_vel);
    pawn_->SetAngularVelocity(controls.angular_vel);
}

msr::airlib::WarthogApiBase::WarthogState WarthogPawnApi::getWarthogState() const
{
    msr::airlib::WarthogApiBase::WarthogState state(pawn_->desired_liner_vel,
        pawn_->desired_angular_vel,
        *pawn_kinematics_);
    return state;
}

void WarthogPawnApi::reset()
{
    vehicle_api_->reset();

    last_controls_ = msr::airlib::CarApiBase::WarthogControls();
    auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps) {
            phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
            phys_comp->SetSimulatePhysics(false);
        }
        //movement_->ResetMoveState();
        //movement_->SetActive(false);
        //movement_->SetActive(true, true);
        vehicle_api_->setWarthogControls(msr::airlib::WarthogApiBase::WarthogControls());
        updateMovement(msr::airlib::WarthogApiBase::WarthogControls());

    },
                                             true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps)
            phys_comp->SetSimulatePhysics(true);
    },
                                             true);
}

void WarthogPawnApi::update()
{
    vehicle_api_->updateWarthogState(getWarthogState());
    vehicle_api_->update();
}

WarthogPawnApi::~WarthogPawnApi() = default;