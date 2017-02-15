#include "AirSim.h"
#include "SimHUD.h"


void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    setupInputBindings();
}

void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::BindActionTokey("InputEventToggleReport", EKeys::R, this, &ASimHUD::InputEventToggleReport);
    UAirBlueprintLib::BindActionTokey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::InputEventToggleHelp);
    UAirBlueprintLib::BindActionTokey("InputEventToggleTrace", EKeys::T, this, &ASimHUD::InputEventToggleTrace);
    
    UAirBlueprintLib::BindActionTokey("InputEventTogglePIPScene", EKeys::Three, this, &ASimHUD::InputEventTogglePIPScene);
    UAirBlueprintLib::BindActionTokey("InputEventTogglePIPDepth", EKeys::One, this, &ASimHUD::InputEventTogglePIPDepth);
    UAirBlueprintLib::BindActionTokey("InputEventTogglePIPSeg", EKeys::Two, this, &ASimHUD::InputEventTogglePIPSeg);
    UAirBlueprintLib::BindActionTokey("InputEventToggleAll", EKeys::Zero, this, &ASimHUD::InputEventToggleAll);
}