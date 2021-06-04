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
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected: //overrides
    virtual void setupClockSpeed() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const override;
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const override;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const override;
    virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const override;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const override;
    virtual void initializeVehiclePawn(APawn* pawn) override;
    virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
        const PawnSimApi::Params& pawn_sim_api_params) const override;
    virtual msr::airlib::VehicleApiBase* getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                       const PawnSimApi* sim_api) const override;

private:
    typedef AFlyingPawn TVehiclePawn;
};
