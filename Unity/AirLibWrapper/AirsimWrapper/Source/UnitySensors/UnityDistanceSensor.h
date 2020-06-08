#pragma once

#include "sensors/distance/DistanceSimple.hpp"
#include "../NedTransform.h"

class UnityDistanceSensor : public msr::airlib::DistanceSimple 
{
private:
	using Vector3r = msr::airlib::Vector3r;
	using VectorMath = msr::airlib::VectorMath;

private:
	std::string vehicle_name_;
	const NedTransform* ned_transform_;

protected:
	virtual msr::airlib::real_T getRayLength(const msr::airlib::Pose& pose) override;

public:
	UnityDistanceSensor(std::string vehicle_name, const NedTransform* ned_transform);
};