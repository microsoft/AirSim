// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "AirSimGameMode.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API AAirSimGameMode : public AGameModeBase
{
    GENERATED_BODY()
    
    virtual void StartPlay() override;
    
    AAirSimGameMode(const FObjectInitializer& ObjectInitializer);
    
private:
    UGameUserSettings* GetGameUserSettings();

};
