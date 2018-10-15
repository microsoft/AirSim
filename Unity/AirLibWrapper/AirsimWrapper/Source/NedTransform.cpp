#include "NedTransform.h"

NedTransform::NedTransform(const AirSimUnity::UnityTransform& global_transform)
	: global_transform_(global_transform)
{
	local_ned_offset_ = AirSimUnity::AirSimVector(0.0f, 0.0f, 0.0f);
}

NedTransform::NedTransform(const NedTransform& global_transform)
	: global_transform_(global_transform.global_transform_)
{
}
