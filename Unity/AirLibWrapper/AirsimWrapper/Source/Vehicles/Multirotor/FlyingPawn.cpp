#include "FlyingPawn.h"
#include "../../UnityUtilities.hpp"
#include "../../PInvokeWrapper.h"

FlyingPawn::FlyingPawn(std::string multirotor_name) : multirotor_name_(multirotor_name)
{
	pawn_events_.getActuatorSignal().connect_member(this, &FlyingPawn::setRotorSpeed);
}

void FlyingPawn::setRotorSpeed(const std::vector<MultirotorPawnEvents::RotorActuatorInfo>& rotor_infos)
{
	for (auto rotor_index = 0; rotor_index < rotor_infos.size(); ++rotor_index)
	{
		SetRotorSpeed(rotor_index, UnityUtilities::Convert_to_UnityRotorInfo(rotor_infos[rotor_index]), multirotor_name_.c_str());
	}
}