#pragma once

#include "CoreMinimal.h"

#include "CarPawn.h"
#include "common/Common.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "SimModeCar.generated.h"


UCLASS()
class AIRSIM_API ASimModeCar : public ASimModeBase
{
    GENERATED_BODY()

public:
    ASimModeCar();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    virtual bool isPaused() const override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;

private:
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;
    typedef ACarPawn TVehiclePawn;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;

private:
    void setupVehiclesAndCamera();
    void initializePauseState();

protected:
    virtual void setupClockSpeed() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;

private:
    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;

    UPROPERTY()
    TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC

    float follow_distance_;

    std::atomic<float> current_clockspeed_;
    std::atomic<TTimeDelta> pause_period_;
    std::atomic<TTimePoint> pause_period_start_;
};
