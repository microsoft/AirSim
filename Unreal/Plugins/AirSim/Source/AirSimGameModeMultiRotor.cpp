#include "AirSim.h"
#include "AirSimGameModeMultiRotor.h"
#include "SimHUD/SimHUDMultiRotor.h"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "controllers/Settings.hpp"

AAirSimGameModeMultiRotor::AAirSimGameModeMultiRotor(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    SetHUD();
}

AAirSimGameModeMultiRotor::~AAirSimGameModeMultiRotor()
{
}

void AAirSimGameModeMultiRotor::SetHUD()
{
    HUDClass = ASimHUDMultiRotor::StaticClass();
}
