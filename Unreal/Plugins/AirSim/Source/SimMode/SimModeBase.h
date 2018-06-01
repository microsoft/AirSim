#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "GameFramework/Actor.h"

#include <string>
#include "CameraDirector.h"
#include "common/AirSimSettings.hpp"
#include "common/ClockFactory.hpp"
#include "api/ApiServerBase.hpp"
#include "api/ApiProvider.hpp"
#include "SimModeBase.generated.h"


UCLASS()
class AIRSIM_API ASimModeBase : public AActor
{
public:

    GENERATED_BODY()

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
    virtual std::string getDebugReport();
    VehicleSimApi* ASimModeBase::getFpvVehicleSimApi();

    virtual ECameraDirectorMode getInitialViewMode() const;

    virtual bool isPaused() const;
    virtual void pause(bool is_paused);
    virtual void continueForTime(double seconds);

    virtual void startRecording();
    virtual void stopRecording();
    virtual bool isRecording() const;

    void startApiServer();
    void stopApiServer();
    bool isApiServerStarted();

protected:
    virtual void setupInputBindings();
    virtual const msr::airlib::AirSimSettings& getSettings() const;
    long long getPhysicsLoopPeriod() const;
    void setPhysicsLoopPeriod(long long  period);
    virtual void setupClockSpeed();

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;
    msr::airlib::ApiProvider* getApiProvider() const
    {
        return api_provider_.get();
    }

protected:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    static const char kUsageScenarioComputerVision[];
    int record_tick_count;

private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;

private:
    UClass* sky_sphere_class_;
    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;;
    TTimePoint tod_sim_clock_start_;
    TTimePoint tod_last_update_;
    std::time_t tod_start_time_;
    long long physics_loop_period_;
    std::unique_ptr<msr::airlib::WorldSimApiBase> world_sim_api_;
    std::unique_ptr<msr::airlib::ApiProvider> api_provider_;
    std::unique_ptr<msr::airlib::ApiServerBase> api_server_;

private:
    void setStencilIDs();
    void setupTimeOfDay();
    void advanceTimeOfDay();
    void setupPhysicsLoopPeriod();
    void showClockStats();
    void checkVehicleReady();
};
