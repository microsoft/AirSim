// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelRear.h"
#include "UObject/ConstructorHelpers.h"

UCarWheelRear::UCarWheelRear()
{
#if ENGINE_MAJOR_VERSION >= 5
    WheelRadius = 18.f;
    WheelWidth = 15.0f;
    bAffectedByHandbrake = true;
    MaxSteerAngle = 0.f;

    // Setup suspension forces
    SuspensionForceOffset = FVector::ZeroVector;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionDampingRatio = 1.05f;

    // Chaos Vehicles do not use UTireConfig
#else
    ShapeRadius = 18.f;
    ShapeWidth = 15.0f;
    bAffectedByHandbrake = true;
    SteerAngle = 0.f;

    // Setup suspension forces
    SuspensionForceOffset = -0.0f;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionNaturalFrequency = 9.0f;
    SuspensionDampingRatio = 1.05f;

    static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/Vehicle/WheelData/Vehicle_BackTireConfig.Vehicle_BackTireConfig"));
    TireConfig = TireData.Object;
#endif
}
