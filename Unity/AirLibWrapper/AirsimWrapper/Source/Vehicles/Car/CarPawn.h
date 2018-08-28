#pragma once

#include "common/AirSimSettings.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "../../UnityPawn.h"

class CarPawn : public UnityPawn
{
private:
	typedef msr::airlib::AirSimSettings AirSimSettings;

private:
	bool is_low_friction_;
	msr::airlib::CarApiBase::CarControls keyboard_controls_;

public:
	CarPawn(std::string car_name);

	const msr::airlib::CarApiBase::CarControls& getKeyBoardControls() const
	{
		return keyboard_controls_;
	}

public:
	std::string car_name_;
};
