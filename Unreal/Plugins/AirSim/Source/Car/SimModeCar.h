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

    virtual VehiclePawnWrapper* getFpvVehiclePawnWrapper() override;

    void createVehicles(std::vector<VehiclePtr>& vehicles);
    virtual void reset() override;
    virtual std::string getReport() override;

private:
    void setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles);
    void updateReport();
    int getRemoteControlID(const VehiclePawnWrapper& pawn);


private:    
    UClass* external_camera_class_;
    UClass* camera_director_class_;
    UClass* vehicle_pawn_class_;

    TArray<AActor*> spawned_actors_;
    std::vector<VehiclePtr> vehicles_;
    VehiclePawnWrapper* fpv_vehicle_pawn_wrapper_;
    float follow_distance_;
    msr::airlib::StateReporterWrapper report_wrapper_;
};
