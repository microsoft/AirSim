#pragma once

#include <memory>
#include <vector>
#include "VehicleConnectorBase.h"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/World.hpp"
#include "common/StateReporterWrapper.hpp"
#include "rpc/ControlServerBase.hpp"
#include "SimModeBase.h"
#include "SimModeWorldBase.generated.h"



UCLASS()
class AIRSIM_API ASimModeWorldBase : public ASimModeBase
{
    GENERATED_BODY()
    
public:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;


    virtual void reset() override;
    virtual std::string getReport() override;
    virtual void setupInputBindings() override;

protected:
    typedef std::shared_ptr<VehicleConnectorBase> VehiclePtr;
    virtual void createVehicles(std::vector<VehiclePtr>& vehicles);
    size_t getVehicleCount() const;

private:
    void createWorld();

private:
    msr::airlib::World world_;
    msr::airlib::FastPhysicsEngine physics_engine_;

    std::vector<VehiclePtr> vehicles_;
    msr::airlib::StateReporterWrapper reporter_;
};
