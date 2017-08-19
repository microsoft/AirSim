#pragma once

#include "GameFramework/HUD.h"
#include "SimHUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "SimHUDMultiRotor.generated.h"


UCLASS()
class AIRSIM_API ASimHUDMultiRotor : public ASimHUD
{
	GENERATED_BODY()

public:
	ASimHUDMultiRotor();
	~ASimHUDMultiRotor();

protected:
	void CreateSimMode() override;
};

