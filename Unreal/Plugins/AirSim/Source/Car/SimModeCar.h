#pragma once

#include "CoreMinimal.h"
#include "common/Common.hpp"
#include "SimMode/SimModeWorldBase.h"
#include "VehiclePawnWrapper.h"
#include "common/StateReporterWrapper.hpp"
#include "SimModeCar.generated.h"


UCLASS()
class AIRSIM_API ASimModeCar : public ASimModeBase
{
    GENERATED_BODY()

public:
    typedef ACarPawn* VehiclePtr;
    typedef ACarPawn TVehiclePawn;

    ASimModeCar();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    virtual VehiclePawnWrapper* getFpvVehiclePawnWrapper() const override;

    void createVehicles(std::vector<VehiclePtr>& vehicles);
    virtual void reset() override;
    virtual std::string getReport() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;

    virtual bool isPaused() const override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTicks(uint32_t ticks) override;

private:
    void setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles);
    void updateReport();
    int getRemoteControlID(const VehiclePawnWrapper& pawn) const;
    void initializePauseState();

protected:
    virtual void setupClockSpeed() override;


private:    
    UClass* external_camera_class_;
    UClass* camera_director_class_;

    TArray<AActor*> spawned_actors_;
    std::vector<VehiclePtr> vehicles_;
    VehiclePawnWrapper* fpv_vehicle_pawn_wrapper_;
    float follow_distance_;
    msr::airlib::StateReporterWrapper report_wrapper_;

    float current_clockspeed_;
    uint32_t pause_countdown_;
    bool pause_countdown_enabled_;
};
