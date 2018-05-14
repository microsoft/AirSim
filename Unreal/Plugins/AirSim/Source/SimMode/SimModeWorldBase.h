#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <vector>
#include "controllers/VehicleConnectorBase.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/World.hpp"
#include "physics/PhysicsWorld.hpp"
#include "common/StateReporterWrapper.hpp"
#include "api/ApiServerBase.hpp"
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

    virtual bool isPaused() const override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;

protected:
    typedef std::shared_ptr<msr::airlib::VehicleConnectorBase> VehiclePtr;
    virtual void createVehicles(std::vector<VehiclePtr>& vehicles);
    size_t getVehicleCount() const;

    UPROPERTY() UManualPoseController* manual_pose_controller;

    void startAsyncUpdator();
    void stopAsyncUpdator();

private:
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::PhysicsEngineBase PhysicsEngineBase;
    typedef msr::airlib::ClockFactory ClockFactory;

    PhysicsEngineBase* createPhysicsEngine();
    static std::vector<UpdatableObject*> toUpdatableObjects(const std::vector<VehiclePtr>& vehicles);

private:
    std::unique_ptr<msr::airlib::PhysicsWorld> physics_world_;
    
    std::unique_ptr<PhysicsEngineBase> physics_engine_;

    std::vector<VehiclePtr> vehicles_;
};
