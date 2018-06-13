#pragma once

#include "CoreMinimal.h"

#include "ComputerVisionPawn.h"
#include "common/Common.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "SimModeComputerVision.generated.h"


UCLASS()
class AIRSIM_API ASimModeComputerVision : public ASimModeBase
{
    GENERATED_BODY()

public:
    ASimModeComputerVision();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;

private:
    typedef AComputerVisionPawn TVehiclePawn;

private:
    void setupVehiclesAndCamera();

protected:
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;

private:
    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC

    float follow_distance_;
};
