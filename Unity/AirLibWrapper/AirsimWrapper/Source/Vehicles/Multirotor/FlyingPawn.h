#pragma once

#include "MultirotorPawnEvents.h"
#include "../../UnityPawn.h"

class FlyingPawn : public UnityPawn
{
public:
	float RotatorFactor = 1.0f;

public:
	FlyingPawn(std::string multirotor_name);
	MultirotorPawnEvents* getPawnEvents()
	{
		return &pawn_events_;
	}
	//called by API to set rotor speed
	void setRotorSpeed(const std::vector<MultirotorPawnEvents::RotorActuatorInfo>& rotor_infos);

private:
	static constexpr size_t rotor_count = 4;
	MultirotorPawnEvents pawn_events_;
	std::string multirotor_name_;
};