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

NedTransform::Quaternionr NedTransform::toNed(const AirSimUnity::AirSimQuaternion& q) const
{
	return Quaternionr(q.w, -q.x, -q.y, q.z);
}

AirSimUnity::AirSimQuaternion NedTransform::fromNed(const Quaternionr& q) const
{
	return AirSimUnity::AirSimQuaternion(-q.x(), -q.y(), q.z(), q.w());
}

AirSimUnity::AirSimVector NedTransform::getLocalOffset() const
{
	return local_ned_offset_;
}

AirSimUnity::UnityTransform NedTransform::getGlobalTransform() const
{
	return global_transform_;
}