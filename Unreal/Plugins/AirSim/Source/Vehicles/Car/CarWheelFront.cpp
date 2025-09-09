// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelFront.h"
#include "UObject/ConstructorHelpers.h"

UCarWheelFront::UCarWheelFront()
{
#if ENGINE_MAJOR_VERSION >= 5
    WheelRadius = 18.f;
    WheelWidth = 15.0f;
    WheelMass = 20.0f;
    bAffectedByHandbrake = false;
    MaxSteerAngle = 40.f;

    SuspensionForceOffset = FVector::ZeroVector;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionDampingRatio = 1.05f;

    // Chaos Vehicles do not use UTireConfig; tune friction via wheel/physmat if needed
#else
    ShapeRadius = 18.f;
    ShapeWidth = 15.0f;
    Mass = 20.0f;
    DampingRate = 0.25f;
    bAffectedByHandbrake = false;
    SteerAngle = 40.f;

    // Setup suspension forces
    SuspensionForceOffset = 0.0f;
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    SuspensionNaturalFrequency = 9.0f;
    SuspensionDampingRatio = 1.05f;

    static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/Vehicle/WheelData/Vehicle_FrontTireConfig.Vehicle_FrontTireConfig"));
    TireConfig = TireData.Object;
#endif
}
