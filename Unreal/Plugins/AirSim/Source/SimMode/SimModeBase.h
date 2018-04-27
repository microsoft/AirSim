#pragma once

#include "CoreMinimal.h"
#include <string>
#include "CameraDirector.h"
#include "GameFramework/Actor.h"
#include "ManualPoseController.h"
#include "VehiclePawnWrapper.h"
#include "common/AirSimSettings.hpp"
#include "Components/SkyLightComponent.h"
#include "common/ClockFactory.hpp"
#include "Engine/DirectionalLight.h"
#include "api/ApiServerBase.hpp"
#include "api/SimModeApiBase.hpp"
#include "SimModeBase.generated.h"


UCLASS()
class AIRSIM_API ASimModeBase : public AActor
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Refs")
    ACameraDirector* CameraDirector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
    bool EnableReport = false;

    UFUNCTION(BlueprintCallable, Category = "Recording")
    bool toggleRecording();

public:	
    // Sets default values for this actor's properties
    ASimModeBase();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick( float DeltaSeconds ) override;

    //additional overridable methods
    virtual void reset();
    virtual std::string getReport();
    virtual void startRecording();
    virtual void stopRecording();
    virtual bool isRecording() const;
    virtual bool isRecordUIVisible() const;
    virtual ECameraDirectorMode getInitialViewMode() const;

    //must be implemented by derived class
    //can't use pure virtual because of restriction with Unreal
    virtual VehiclePawnWrapper* getFpvVehiclePawnWrapper() const;
    virtual msr::airlib::VehicleApiBase* getVehicleApi() const;

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;

    virtual bool isPaused() const;
    virtual void pause(bool is_paused);
    virtual void continueForTime(double seconds);

protected:
    typedef msr::airlib::AirSimSettings AirSimSettings;
    virtual void setupInputBindings();
    virtual const AirSimSettings& getSettings() const;
    long long getPhysicsLoopPeriod() const;
    void setPhysicsLoopPeriod(long long  period);
    msr::airlib::SimModeApiBase* getSimModeApi() const;
    virtual void setupClockSpeed();

protected: //settings
    int record_tick_count;
    static const char kUsageScenarioComputerVision[];


private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;


    class SimModeApi : public msr::airlib::SimModeApiBase  {
    public:
        SimModeApi(ASimModeBase* simmode);
        virtual msr::airlib::VehicleApiBase* getVehicleApi() override;
        virtual void reset() override;
        virtual bool isPaused() const override;
        virtual void pause(bool is_paused) override;
        virtual void continueForTime(double seconds) override;

    private:
        ASimModeBase* simmode_;
    };

private:
    UClass* sky_sphere_class_;
    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;;
    TTimePoint tod_sim_clock_start_;
    TTimePoint tod_last_update_;
    std::time_t tod_start_time_;
    long long physics_loop_period_;
    std::unique_ptr<SimModeApi> simmode_api_;

private:
    void setStencilIDs();
    void setupTimeOfDay();
    void advanceTimeOfDay();
    void setupPhysicsLoopPeriod();
    void showClockStats();

};
