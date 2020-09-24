#pragma once

#include "CoreMinimal.h"
#include "Engine/SceneCapture2D.h"
#include "DistortableSceneCapture.generated.h"

UCLASS()
class ADistortableSceneCapture : public ASceneCapture2D
{
    GENERATED_BODY()

public:
    UPROPERTY(EditDefaultsOnly, Category = "Distortion")
    UMaterialParameterCollection *ParamCollection;
};