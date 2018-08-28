#include "CarPawn.h"

#define LOCTEXT_NAMESPACE "VehiclePawn"

CarPawn::CarPawn(std::string car_name) : car_name_(car_name)
{
	is_low_friction_ = false;
}

#undef LOCTEXT_NAMESPACE
