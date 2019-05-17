#include "CarPawnApi.h"
#include "AirBlueprintLib.h"

#include "PhysXVehicleManager.h"

CarPawnApi::CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
    const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory, 
    const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment)
    : msr::airlib::CarApiBase(vehicle_setting, sensor_factory, state, environment),
    pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint)
{
    movement_ = pawn->GetVehicleMovement();
}

bool CarPawnApi::armDisarm(bool arm)
{
    //TODO: implement arming for car
    unused(arm);
    return true;
}

void CarPawnApi::setCarControls(const CarApiBase::CarControls& controls)
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

const msr::airlib::CarApiBase::CarControls& CarPawnApi::getCarControls() const
{
    return last_controls_;
}

msr::airlib::CarApiBase::CarState CarPawnApi::getCarState() const
{
    CarApiBase::CarState state(
        movement_->GetForwardSpeed() / 100, //cm/s -> m/s
        movement_->GetCurrentGear(),
        movement_->GetEngineRotationSpeed(),
        movement_->GetEngineMaxRotationSpeed(),
        last_controls_.handbrake,
        *pawn_kinematics_,
        msr::airlib::ClockFactory::get()->nowNanos()
    );
    return state;
}

void CarPawnApi::resetImplementation()
{
    msr::airlib::CarApiBase::resetImplementation();

    last_controls_ = CarControls();
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
        setCarControls(CarControls());

	auto pv = movement_->PVehicle;
	if (pv) {
	  pv->mWheelsDynData.setToRestState();
	}
	auto pvd = movement_->PVehicleDrive;
	if (pvd) {
	  pvd->mDriveDynData.setToRestState();
	}
    }, true);

    UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
        for (auto* phys_comp : phys_comps)
            phys_comp->SetSimulatePhysics(true);
    }, true);
}

void CarPawnApi::update()
{
    msr::airlib::CarApiBase::update();
}

msr::airlib::GeoPoint CarPawnApi::getHomeGeoPoint() const
{
    return home_geopoint_;
}

void CarPawnApi::enableApiControl(bool is_enabled)
{
    if (api_control_enabled_ != is_enabled) {
        last_controls_ = CarControls();
        api_control_enabled_ = is_enabled;
    }
}

bool CarPawnApi::isApiControlEnabled() const
{
    return api_control_enabled_;
}

CarPawnApi::~CarPawnApi() = default;
