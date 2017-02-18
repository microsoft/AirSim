#pragma once

#include "SimModeBase.h"
#include "Blueprint/UserWidget.h"
#include "SimHUDWidget.generated.h"


UCLASS()
class AIRSIM_API USimHUDWidget : public UUserWidget {
    GENERATED_BODY()

  public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Refs")
    ASimModeBase* SimMode;
};
