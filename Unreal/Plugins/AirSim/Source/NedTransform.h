#pragma once

#include "CoreMinimal.h"
#include "common/Common.hpp"
#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Actor.h"


class NedTransform
{
public:
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;

public:
    NedTransform();
    void initialize(const AActor* pivot);
    bool isInitialized() const;
    Vector3r toNedMeters(const FVector& position, bool use_offset = true) const;
    FVector toNeuUU(const Vector3r& position) const;
    FQuat toFQuat(const Quaternionr& q, bool convert_from_ned) const;
    Quaternionr toQuaternionr(const FQuat& q, bool convert_to_ned) const;
    float toNedMeters(float length) const;
    float toNeuUU(float length) const;

private:
    FVector toFVector(const Vector3r& vec, float scale, bool convert_from_ned) const;
    Vector3r toVector3r(const FVector& vec, float scale, bool convert_to_ned) const;

private:
    FVector offset_;
    float world_to_meters_;
    bool is_initialized_;
};
