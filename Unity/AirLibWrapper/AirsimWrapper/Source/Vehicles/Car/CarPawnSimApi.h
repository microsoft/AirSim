#pragma once

#include "CarPawn.h"
#include "CarPawnApi.h"
#include "../../PawnSimApi.h"
#include "vehicles/car/api/CarApiBase.hpp"

class CarPawnSimApi : public PawnSimApi
{
public:
	typedef msr::airlib::Utils Utils;
	typedef msr::airlib::StateReporter StateReporter;
	typedef msr::airlib::UpdatableObject UpdatableObject;
	typedef msr::airlib::Pose Pose;

private:
	void createVehicleApi(CarPawn* pawn, const msr::airlib::GeoPoint& home_geopoint);
	void updateCarControls();

public:
    virtual void initialize() override;
	CarPawnSimApi(const Params& params, const CarPawnApi::CarControls&  keyboard_controls, std::string car_name);
	virtual ~CarPawnSimApi() = default;
	virtual void resetImplementation() override;
	virtual void update() override;
	//virtual void reportState(StateReporter& reporter) override;
	virtual std::string getRecordFileLine(bool is_header_line) const override;
	virtual void updateRenderedState(float dt) override;
	virtual void updateRendering(float dt) override;

	msr::airlib::CarApiBase* getVehicleApi() const
	{
		return vehicle_api_.get();
	}

private:
    Params params_;

	std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
	std::vector<std::string> vehicle_api_messages_;
	CarPawnApi::CarControls joystick_controls_;
	CarPawnApi::CarControls current_controls_;
	std::string car_name_;

	//storing reference from pawn
	const CarPawnApi::CarControls& keyboard_controls_;
};