// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "Car4x4WheelRear.h"
#include "TireConfig.h"
#include "UObject/ConstructorHelpers.h"

UCar4x4WheelRear::UCar4x4WheelRear()
{
	ShapeRadius = 18.0f;
	ShapeWidth = 15.0f;
	bAffectedByHandbrake = true;
	SteerAngle = 0.f;

	// Setup suspension forces
	SuspensionForceOffset = -4.0f;
	SuspensionMaxRaise = 8.0f;
	SuspensionMaxDrop = 12.0f;
	SuspensionNaturalFrequency = 9.0f;
	SuspensionDampingRatio = 1.05f;

	// Find the tire object and set the data for it
	static ConstructorHelpers::FObjectFinder<UTireConfig> TireData(TEXT("/Game/CarPhysXAdv/Vehicle/WheelData/Vehicle_BackTireConfig.Vehicle_BackTireConfig"));
	TireConfig = TireData.Object;
}
