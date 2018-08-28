#include "UnityDistanceSensor.h"

UnityDistanceSensor::UnityDistanceSensor(std::string vehicle_name, const NedTransform* ned_transform)
	:vehicle_name_(vehicle_name), ned_transform_(ned_transform)
{
}

msr::airlib::real_T UnityDistanceSensor::getRayLength(const msr::airlib::Pose& pose)
{
	//update ray tracing
	Vector3r start = pose.position;
	Vector3r end = start + VectorMath::rotateVector(VectorMath::front(), pose.orientation, true) * getParams().max_distance;

	//TODO Ankit - need to PInvoke

	//FHitResult dist_hit = FHitResult(ForceInit);
	//bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), dist_hit);
	//float distance = is_hit ? dist_hit.Distance / 100.0f : getParams().max_distance;

	return 0.1f;
}
