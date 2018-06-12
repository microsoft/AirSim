#pragma once

#include "CoreMinimal.h"

#include "FlyingPawn.h"
#include "common/Common.hpp"
#include "SimMode/SimModeWorldBase.h"
#include "api/VehicleSimApiBase.hpp"
#include "SimModeWorldMultiRotor.generated.h"


UCLASS()
class AIRSIM_API ASimModeWorldMultiRotor : public ASimModeWorldBase
{
    GENERATED_BODY()

public:
    ASimModeWorldMultiRotor();
    virtual void BeginPlay() override;

    virtual void Tick( float DeltaSeconds ) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
    virtual void setupClockSpeed() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;

private:
    void setupVehiclesAndCamera();

private:    
    typedef AFlyingPawn TVehiclePawn;

    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;
    float follow_distance_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC
};
