#pragma once

#include "CoreMinimal.h"
#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Actor.h"

#include "common/Common.hpp"

/*
    Note on coordinate system
    -------------------------
    We have following coordinate systems:
    (1) UU or Unreal Units or Unreal Coordinate system
    (2) Global NED: This is NED transformation of UU with origin set to 0,0,0
    (3) Local NED: This is NED transformation of UU with origin set to vehicle's spawning UU location
    (4) Geo: This is GPS coordinate where UU origin is assigned some geo-coordinate

    Vehicles are spawned at position specified in settings in global NED
*/

class AIRSIM_API NedTransform
{
public:
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::Pose Pose;

public:
    NedTransform(const FTransform& global_transform, float world_to_meters);
    NedTransform(const AActor* pivot, const NedTransform& global_transform);

    //UU -> local NED
    Vector3r toLocalNed(const FVector& position) const;
    Vector3r toGlobalNed(const FVector& position) const;
    Quaternionr toNed(const FQuat& q) const;
    float toNed(float length) const;
    Pose toLocalNed(const FTransform& pose) const;
    Pose toGlobalNed(const FTransform& pose) const;


    //local NED -> UU
    FVector fromLocalNed(const Vector3r& position) const;
    FVector fromGlobalNed(const Vector3r& position) const;
    FQuat fromNed(const Quaternionr& q) const;
    float fromNed(float length) const;
    FTransform fromLocalNed(const Pose& pose) const;
    FTransform fromGlobalNed(const Pose& pose) const;

    FVector getGlobalOffset() const;
    FVector getLocalOffset() const;
    FTransform getGlobalTransform() const;

private:
    NedTransform(const AActor* pivot, const FTransform& global_transform, float world_to_meters); //create only through static factory methods
    FVector toFVector(const Vector3r& vec, float scale, bool convert_from_ned) const;
    Vector3r toVector3r(const FVector& vec, float scale, bool convert_to_ned) const;

private:
    FTransform global_transform_;
    float world_to_meters_;
    FVector local_ned_offset_;
};
