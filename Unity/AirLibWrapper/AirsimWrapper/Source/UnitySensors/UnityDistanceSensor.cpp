#include "UnityDistanceSensor.h"
#include "../AirSimStructs.hpp"
#include "../PInvokeWrapper.h"
#include "../UnityUtilities.hpp"

UnityDistanceSensor::UnityDistanceSensor(std::string vehicle_name, const NedTransform* ned_transform)
	:vehicle_name_(vehicle_name), ned_transform_(ned_transform)
{
}

msr::airlib::real_T UnityDistanceSensor::getRayLength(const msr::airlib::Pose& pose)
{
	//update ray tracing
	Vector3r start = pose.position;
	Vector3r end = start + VectorMath::rotateVector(VectorMath::front(), pose.orientation, true) * getParams().max_distance;

	AirSimUnity::RayCastHitResult hitResult = GetRayCastHit(UnityUtilities::Convert_to_AirSimVector(start), 
		UnityUtilities::Convert_to_AirSimVector(end), vehicle_name_.c_str());
	
	float distance = hitResult.isHit ? hitResult.distance : getParams().max_distance;
	
	return distance;
}
