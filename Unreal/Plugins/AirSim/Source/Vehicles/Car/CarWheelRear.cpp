// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarWheelRear.h"
//#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"

UCarWheelRear::UCarWheelRear()
{
    WheelRadius = 38.f;
    WheelWidth = 17.0f;
    bAffectedByHandbrake = true;
    MaxSteerAngle = 0.f;
    AxleType = EAxleType::Rear;

    // Setup suspension forces
    SuspensionForceOffset = FVector(0.0f, 0.0f, 0.0f);
    SuspensionMaxRaise = 10.0f;
    SuspensionMaxDrop = 10.0f;
    //SuspensionNaturalFrequency = 9.0f;
    SuspensionDampingRatio = 1.5f;
}
