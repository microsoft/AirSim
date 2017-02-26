#pragma once

#include "common/Common.hpp"
#include "MavMultiRotorConnector.h"
#include "SimModeWorldBase.h"
#include "SimModeWorldMultiRotor.generated.h"


UCLASS()
class AIRSIM_API ASimModeWorldMultiRotor : public ASimModeWorldBase
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void Tick( float DeltaSeconds ) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    bool checkConnection();
    VehiclePtr createVehicle(AFlyingPawn* pawn);

private:
    std::shared_ptr<VehicleConnectorBase> fpv_vehicle_connector_;
    TArray<uint8> image_;

};
