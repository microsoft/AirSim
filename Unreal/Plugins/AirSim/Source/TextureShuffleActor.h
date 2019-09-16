#pragma once

#include "CoreMinimal.h"
#include "Materials/Material.h"
#include "common/common_utils/Utils.hpp"
#include "common/AirSimSettings.hpp"
#include "Engine/StaticMeshActor.h"
#include "TextureShuffleActor.generated.h"


UCLASS()
class AIRSIM_API ATextureShuffleActor : public AStaticMeshActor
{
	GENERATED_BODY()
protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TextureShuffle)
	UTexture2D* DefaultTexture;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TextureShuffle)
	TArray<UTexture2D*> SwappableTextures;

public:

	UFUNCTION(BlueprintImplementableEvent)
	void SwapTexture(FString tag, int tex_id = 0);
};