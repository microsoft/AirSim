#pragma once

#include "common/Common.hpp"
#include "control/RpcLibServer.hpp"
#include "MavMultiRotor.h"
#include "SimModeWorldBase.h"
#include "SimModeWorldMultiRotor.generated.h"

UCLASS()
class AIRSIM_API ASimModeWorldMultiRotor : public ASimModeWorldBase {
    GENERATED_BODY()

  public:
    virtual void BeginPlay() override;
    virtual void Tick( float DeltaSeconds ) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  protected:
    virtual void createVehicles(std::vector<VehiclePtr>& vehicles) override;
    bool checkConnection();

  private:
    std::shared_ptr<MavMultiRotor> fpv_vehicle_;
    std::unique_ptr<msr::airlib::DroneControlServer> drone_control_server_;
    std::unique_ptr<msr::airlib::RpcLibServer> rpclib_server_;
    TArray<uint8> image_;

};
