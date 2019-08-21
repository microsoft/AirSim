#include "CarPawnApi.h"
#include "../../PInvokeWrapper.h"


CarPawnApi::CarPawnApi(CarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics, const msr::airlib::GeoPoint& home_geopoint,
	const msr::airlib::AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<msr::airlib::SensorFactory> sensor_factory,
	const std::string car_name,
	const msr::airlib::Kinematics::State& state, const msr::airlib::Environment& environment)
	: msr::airlib::CarApiBase(vehicle_setting, sensor_factory, state, environment),
	pawn_(pawn), pawn_kinematics_(pawn_kinematics), home_geopoint_(home_geopoint), car_name_(car_name)
{
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
	SetCarApiControls(controls, car_name_.c_str());
}

const msr::airlib::CarApiBase::CarControls& CarPawnApi::getCarControls() const
{
	return last_controls_;
}

msr::airlib::CarApiBase::CarState CarPawnApi::getCarState() const
{
	AirSimCarState carState = GetCarState(car_name_.c_str());

	CarApiBase::CarState state(
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

void CarPawnApi::resetImplementation()
{
	msr::airlib::CarApiBase::resetImplementation();

	last_controls_ = CarControls();
	setCarControls(CarControls());
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
		SetEnableApi(is_enabled, car_name_.c_str());
	}
}

bool CarPawnApi::isApiControlEnabled() const
{
	return api_control_enabled_;
}

CarPawnApi::~CarPawnApi() = default;
