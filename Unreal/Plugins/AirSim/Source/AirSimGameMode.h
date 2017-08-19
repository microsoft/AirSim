#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "AirSimGameMode.generated.h"

UCLASS()
class AIRSIM_API AAirSimGameMode : public AGameModeBase
{
    GENERATED_BODY()
    virtual void StartPlay() override;

protected:
    AAirSimGameMode(const FObjectInitializer& ObjectInitializer);
    virtual void SetHUD();
    
};
