#include "CarPawnApi.h"
#include "../../PInvokeWrapper.h"


CarPawnApi::CarPawnApi(CarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
	const std::string car_name, msr::airlib::CarApiBase* vehicle_api)
	: pawn_(pawn), pawn_kinematics_(pawn_kinematics), car_name_(car_name), vehicle_api_(vehicle_api)
{
}

void CarPawnApi::updateMovement(const msr::airlib::CarApiBase::CarControls& controls)
{
	last_controls_ = controls;
	SetCarApiControls(controls, car_name_.c_str());
}

msr::airlib::CarApiBase::CarState CarPawnApi::getCarState() const
{
	AirSimCarState carState = GetCarState(car_name_.c_str());

	msr::airlib::CarApiBase::CarState state(
		carState.speed,
		carState.gear,
		carState.engineRotationSpeed,
		carState.engineMaxRotationSpeed,
		last_controls_.handbrake,
		*pawn_kinematics_,
		msr::airlib::ClockFactory::get()->nowNanos()
	);

	return state;
}

void CarPawnApi::reset()
{
	vehicle_api_->reset();

	last_controls_ = msr::airlib::CarApiBase::CarControls();
	updateMovement(msr::airlib::CarApiBase::CarControls());
}

void CarPawnApi::update()
{
	vehicle_api_->updateCarState(getCarState());
	vehicle_api_->update();
}

CarPawnApi::~CarPawnApi() = default;
