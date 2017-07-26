#include "NedTransform.h"
#include "AirBlueprintLib.h"


FVector NedTransform::offset_ = FVector::ZeroVector;
float NedTransform::world_to_meters_ = 100.0f;
bool NedTransform::is_initialized_ = false;

void NedTransform::initialize(const AActor* pivot)
{
    FVector mesh_origin, mesh_bounds;
    pivot->GetActorBounds(true, mesh_origin, mesh_bounds);

    FVector ground_offset = FVector(0, 0, mesh_bounds.Z);
    offset_ = pivot->GetActorLocation() - ground_offset;

    world_to_meters_ = UAirBlueprintLib::GetWorldToMetersScale(pivot);

    is_initialized_ = true;
}

bool NedTransform::isInitialized()
{
    return is_initialized_;
}

float NedTransform::toNedMeters(float length)
{
    return length / world_to_meters_;
}

NedTransform::Vector3r NedTransform::toNedMeters(const FVector& position, bool use_offset)
{
    return NedTransform::toVector3r(position - (use_offset ? offset_ : FVector::ZeroVector), 1 / world_to_meters_, true);
}

FVector NedTransform::toNeuUU(const NedTransform::Vector3r& position)
{
    return NedTransform::toFVector(position, world_to_meters_, true) + offset_;
}

FVector NedTransform::toFVector(const Vector3r& vec, float scale, bool convert_from_ned)
{
    return FVector(vec.x() * scale, vec.y() * scale, 
        (convert_from_ned ? -vec.z() : vec.z()) * scale);
}

FQuat NedTransform::toFQuat(const Quaternionr& q, bool convert_from_ned)
{
    return FQuat(
        convert_from_ned ? -q.x() : q.x(), 
        convert_from_ned ? -q.y() : q.y(), 
        q.z(), q.w());
}


NedTransform::Vector3r NedTransform::toVector3r(const FVector& vec, float scale, bool convert_to_ned)
{
    return Vector3r(vec.X * scale, vec.Y * scale, 
        (convert_to_ned ? -vec.Z : vec.Z)  * scale);
}

NedTransform::Quaternionr NedTransform::toQuaternionr(const FQuat& q, bool convert_to_ned)
{
    return Quaternionr(q.W, 
        convert_to_ned ? -q.X : q.X, 
        convert_to_ned ? -q.Y : q.Y, 
        q.Z);
}