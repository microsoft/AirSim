#pragma once

#include "AirsimLevelStreaming.h"
#include "Engine/World.h"

int32 UAirsimLevelStreaming::LevelInstanceId = 0;

UAirsimLevelStreaming* UAirsimLevelStreaming::LoadAirsimLevelInstance(UWorld* WorldContextObject, FString LevelName, FVector Location, FRotator Rotation, bool& bOutSuccess)
{
    if (!WorldContextObject) {
        return nullptr;
    }

    // Check whether requested map exists, this could be very slow if LevelName is a short package name
    FString LongPackageName;
    bool success = FPackageName::SearchForPackageOnDisk(LevelName, &LongPackageName);
    if (!success) {
        bOutSuccess = false;
        return nullptr;
    }

    // Create Unique Name for sub-level package
    const FString ShortPackageName = FPackageName::GetShortName(LongPackageName);
    const FString PackagePath = FPackageName::GetLongPackagePath(LongPackageName);
    FString UniqueLevelPackageName = PackagePath + TEXT("/") + WorldContextObject->StreamingLevelsPrefix + ShortPackageName;
    UniqueLevelPackageName += TEXT("_LevelInstance_") + FString::FromInt(++LevelInstanceId);

    // Setup streaming level object that will load specified map
    ULevelStreamingDynamic* level_pointer = NewObject<ULevelStreamingDynamic>(WorldContextObject, ULevelStreamingDynamic::StaticClass(), NAME_None, RF_Transient, NULL);
    level_pointer->SetWorldAssetByPackageName(FName(*UniqueLevelPackageName));
    level_pointer->LevelColor = FColor::MakeRandomColor();
    level_pointer->SetShouldBeLoaded(true);
    level_pointer->SetShouldBeVisible(true);
    level_pointer->bShouldBlockOnLoad = true;
    level_pointer->bInitiallyLoaded = true;
    level_pointer->bInitiallyVisible = true;

    // Transform
    level_pointer->LevelTransform = FTransform(Rotation, Location);
    // Map to Load
    level_pointer->PackageNameToLoad = FName(*LongPackageName);
    // Add the new level to world.
    WorldContextObject->AddStreamingLevel(level_pointer);

    bOutSuccess = true;

    return dynamic_cast<UAirsimLevelStreaming*>(level_pointer);
}