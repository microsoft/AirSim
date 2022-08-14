// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelFront.h"
//#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"
#include <ChaosVehicles/ChaosVehiclesCore/Public/WheelSystem.h>

UCarWheelFront::UCarWheelFront()
{
    WheelRadius = 18.f;
    WheelWidth = 15.0f;
    //WheelMass = 20.0f;
    SuspensionDampingRatio = 0.25f;
    bAffectedByHandbrake = false;
    //SteerAngle = 40.f;

    // Setup suspension forces
    SuspensionForceOffset = FVector(0.0f, 0.0f, 0.0f);
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    //SuspensionNaturalFrequency = 9.0f;
    SuspensionDampingRatio = 1.05f;

    // Find the tire object and set the data for it
    //static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/AirSim/VehicleAdv/Vehicle/WheelData/Vehicle_FrontTireConfig.Vehicle_FrontTireConfig"));
    //TireConfig = TireData.Object;
}
