#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "AirSimGameModeBase.generated.h"

UCLASS()
class AIRSIM_API AAirSimGameModeBase : public AGameModeBase
{
    GENERATED_BODY()
    
    virtual void StartPlay() override;

protected:
    AAirSimGameModeBase(const FObjectInitializer& ObjectInitializer);
    virtual void SetHUD();
    
};
