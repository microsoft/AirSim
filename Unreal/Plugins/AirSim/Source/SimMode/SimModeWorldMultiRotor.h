#pragma once

#include "CoreMinimal.h"
#include "common/Common.hpp"
#include "MultiRotorConnector.h"
#include "vehicles/MultiRotorParams.hpp"
#include "SimModeWorldBase.h"
#include "FlyingPawn.h"
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
    AVehiclePawnBase* getFpvVehiclePawn() override;

protected:
    typedef AFlyingPawn TMultiRotorPawn;

    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    bool checkConnection();
    VehiclePtr createVehicle(TMultiRotorPawn* vehicle_pawn);

private:
    void setupVehiclesAndCamera(std::vector<VehiclePtr>& vehicles);

private:    

    TArray<uint8> image_;
    std::vector <std::unique_ptr<msr::airlib::MultiRotorParams> > vehicle_params_;
    bool isLoggingStarted;

    UClass* external_camera_class_;
    UClass* camera_director_class_;
    UClass* vehicle_pawn_class_;

    TArray<AActor*> spawned_actors_;

    AVehiclePawnBase* fpv_vehicle_pawn_;
    std::shared_ptr<VehicleConnectorBase> fpv_vehicle_connector_;
};
