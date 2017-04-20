#pragma once

#include "common/Common.hpp"
#include "MultiRotorConnector.h"
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
	std::shared_ptr<VehicleConnectorBase> fpv_vehicle_connector_;

protected:
    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    bool checkConnection();
    VehiclePtr createVehicle(AFlyingPawn* pawn);

private:    
    TArray<uint8> image_;
	bool isLoggingStarted;
};
