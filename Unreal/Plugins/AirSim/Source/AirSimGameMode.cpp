// Fill out your copyright notice in the Description page of Project Settings.

#include "AirSim.h"
#include "AirSimGameMode.h"
#include "SimHUD/SimHUD.h"



AAirSimGameMode::AAirSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = nullptr;
    HUDClass = ASimHUD::StaticClass();
}

void AAirSimGameMode::StartPlay() 
{
    Super::StartPlay();
}