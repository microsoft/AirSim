#pragma once

#include "GameFramework/GameModeBase.h"
#include "AirSimGameModeBase.h"
#include "AirSimGameModeMultiRotor.generated.h"

UCLASS()
class AIRSIM_API AAirSimGameModeMultiRotor : public AAirSimGameModeBase
{
    GENERATED_BODY()

private:
	AAirSimGameModeMultiRotor(const FObjectInitializer& ObjectInitializer);
	~AAirSimGameModeMultiRotor();
    void SetHUD() override;
};
