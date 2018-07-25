// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelRear.h"
#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"

UCarWheelRear::UCarWheelRear()
{
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

    // Find the tire object and set the data for it
    static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/Vehicle/WheelData/Vehicle_BackTireConfig.Vehicle_BackTireConfig"));
    TireConfig = TireData.Object;
}
