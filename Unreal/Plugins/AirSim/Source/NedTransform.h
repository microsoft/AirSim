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
    static void initialize(const AActor* pivot);
    static bool isInitialized();
    static Vector3r toNedMeters(const FVector& position, bool use_offset = true);
    static FVector toNeuUU(const Vector3r& position);
    static FQuat toFQuat(const Quaternionr& q, bool convert_from_ned);
    static Quaternionr toQuaternionr(const FQuat& q, bool convert_to_ned);
    static float toNedMeters(float length);

    //TODO: make below private
    static FVector toFVector(const Vector3r& vec, float scale, bool convert_from_ned);
    static Vector3r toVector3r(const FVector& vec, float scale, bool convert_to_ned);

private:
    static FVector offset_;
    static float world_to_meters_;
    static bool is_initialized_;
};
