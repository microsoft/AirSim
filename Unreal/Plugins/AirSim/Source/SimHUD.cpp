#include "AirSim.h"
#include "SimHUD.h"


void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    setupInputBindings();
}

void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::BindActionToKey("InputEventToggleReport", EKeys::R, this, &ASimHUD::InputEventToggleReport);
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::InputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::InputEventToggleTrace);
    
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPScene", EKeys::Three, this, &ASimHUD::InputEventTogglePIPScene);
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPDepth", EKeys::One, this, &ASimHUD::InputEventTogglePIPDepth);
    UAirBlueprintLib::BindActionToKey("InputEventTogglePIPSeg", EKeys::Two, this, &ASimHUD::InputEventTogglePIPSeg);
    UAirBlueprintLib::BindActionToKey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::InputEventToggleAll);
}