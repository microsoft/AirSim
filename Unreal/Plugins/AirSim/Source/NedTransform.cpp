#include "NedTransform.h"
#include "AirBlueprintLib.h"

NedTransform::NedTransform(const FTransform& global_transform, float world_to_meters)
    : NedTransform(nullptr, global_transform, world_to_meters)
{
}
NedTransform::NedTransform(const AActor* pivot, const NedTransform& global_transform)
    : NedTransform(pivot, global_transform.global_transform_, global_transform.world_to_meters_)
{
}
NedTransform::NedTransform(const AActor* pivot, const FTransform& global_transform, float world_to_meters)
    : global_transform_(global_transform), world_to_meters_(world_to_meters)
{
    if (pivot != nullptr) {
        //normally pawns have their center as origin. If we use this as 0,0,0 in NED then
        //when we tell vehicle to go to 0,0,0 - it will try to go in the ground
        //so we get the bounds and subtract z to get bottom as 0,0,0
        FVector mesh_origin, mesh_bounds;
        pivot->GetActorBounds(true, mesh_origin, mesh_bounds);

        FVector ground_offset = FVector(0, 0, mesh_bounds.Z);
        local_ned_offset_ = pivot->GetActorLocation() - ground_offset;
    }
    else
        local_ned_offset_ = FVector::ZeroVector;
}

NedTransform::Vector3r NedTransform::toLocalNed(const FVector& position) const
{
    return NedTransform::toVector3r(position - local_ned_offset_,
                                    1 / world_to_meters_,
                                    true);
}
NedTransform::Vector3r NedTransform::toLocalNedVelocity(const FVector& velocity) const
{
    return NedTransform::toVector3r(velocity,
                                    1 / world_to_meters_,
                                    true);
}
NedTransform::Vector3r NedTransform::toGlobalNed(const FVector& position) const
{
    return NedTransform::toVector3r(position - global_transform_.GetLocation(),
                                    1 / world_to_meters_,
                                    true);
}
NedTransform::Quaternionr NedTransform::toNed(const FQuat& q) const
{
    return Quaternionr(q.W, -q.X, -q.Y, q.Z);
}
float NedTransform::toNed(float length) const
{
    return length / world_to_meters_;
}
NedTransform::Pose NedTransform::toLocalNed(const FTransform& pose) const
{
    return Pose(toLocalNed(pose.GetLocation()), toNed(pose.GetRotation()));
}
NedTransform::Pose NedTransform::toGlobalNed(const FTransform& pose) const
{
    return Pose(toGlobalNed(pose.GetLocation()), toNed(pose.GetRotation()));
}

float NedTransform::fromNed(float length) const
{
    return length * world_to_meters_;
}
FVector NedTransform::fromLocalNed(const NedTransform::Vector3r& position) const
{
    return NedTransform::toFVector(position, world_to_meters_, true) + local_ned_offset_;
}
FVector NedTransform::fromRelativeNed(const NedTransform::Vector3r& position) const
{
    return NedTransform::toFVector(position, world_to_meters_, true);
}
FTransform NedTransform::fromRelativeNed(const Pose& pose) const
{
    return FTransform(fromNed(pose.orientation), fromRelativeNed(pose.position));
}
FVector NedTransform::fromGlobalNed(const NedTransform::Vector3r& position) const
{
    return NedTransform::toFVector(position, world_to_meters_, true) + global_transform_.GetLocation();
}
FQuat NedTransform::fromNed(const Quaternionr& q) const
{
    return FQuat(-q.x(), -q.y(), q.z(), q.w());
}
FTransform NedTransform::fromLocalNed(const Pose& pose) const
{
    return FTransform(fromNed(pose.orientation), fromLocalNed(pose.position));
}
FTransform NedTransform::fromGlobalNed(const Pose& pose) const
{
    return FTransform(fromNed(pose.orientation), fromGlobalNed(pose.position));
}
FQuat NedTransform::fromUUtoFLU(const FQuat& q) const
{
    return FQuat(-q.X, q.Y, -q.Z, q.W);
}
// todo unused. need to manually plots tf axes' line in right handed FLU instead of using DrawDebugCoordinateSystem
FQuat NedTransform::fromGlobalNedToFLU(const Quaternionr& q) const
{
    return fromUUtoFLU(fromNed(q));
}

FVector NedTransform::getGlobalOffset() const
{
    return global_transform_.GetLocation();
}
FVector NedTransform::getLocalOffset() const
{
    return local_ned_offset_;
}
FTransform NedTransform::getGlobalTransform() const
{
    return global_transform_;
}

FVector NedTransform::toFVector(const Vector3r& vec, float scale, bool convert_from_ned) const
{
    return FVector(vec.x() * scale, vec.y() * scale, (convert_from_ned ? -vec.z() : vec.z()) * scale);
}

NedTransform::Vector3r NedTransform::toVector3r(const FVector& vec, float scale, bool convert_to_ned) const
{
    return Vector3r(vec.X * scale, vec.Y * scale, (convert_to_ned ? -vec.Z : vec.Z) * scale);
}
