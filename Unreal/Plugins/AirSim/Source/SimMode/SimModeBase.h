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
#include "PawnSimApi.h"
#include "common/StateReporterWrapper.hpp"

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

    const NedTransform& getGlobalNedTransform();

    msr::airlib::ApiProvider* getApiProvider() const
    {
        return api_provider_.get();
    }

    const PawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "") const
    {
        return static_cast<PawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }
    PawnSimApi* getVehicleSimApi(const std::string& vehicle_name = "")
    {
        return static_cast<PawnSimApi*>(api_provider_->getVehicleSimApi(vehicle_name));
    }

protected:
    virtual void setupInputBindings();
    virtual const msr::airlib::AirSimSettings& getSettings() const;
    long long getPhysicsLoopPeriod() const;
    void setPhysicsLoopPeriod(long long  period);
    //called when simmode should do something about clock speed
    virtual void setupClockSpeed();

    void initializeCameraDirector(const FTransform& camera_transform, float follow_distance);
    void checkVehicleReady(); //checks if vehicle is available to use
    virtual void updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter);

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;

protected:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    int record_tick_count;

    UPROPERTY() UClass* pip_camera_class;
    UPROPERTY() UParticleSystem* collision_display_template;
private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;

private:
    //assets loaded in constructor
    UPROPERTY() UClass* external_camera_class_;
    UPROPERTY() UClass* camera_director_class_;
    UPROPERTY() UClass* sky_sphere_class_;


    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;;
    TTimePoint tod_sim_clock_start_;
    TTimePoint tod_last_update_;
    std::time_t tod_start_time_;
    long long physics_loop_period_;
    std::unique_ptr<NedTransform> global_ned_transform_;
    std::unique_ptr<msr::airlib::WorldSimApiBase> world_sim_api_;
    std::unique_ptr<msr::airlib::ApiProvider> api_provider_;
    std::unique_ptr<msr::airlib::ApiServerBase> api_server_;
    msr::airlib::StateReporterWrapper debug_reporter_;

private:
    void setStencilIDs();
    void setupTimeOfDay();
    void advanceTimeOfDay();
    void setupPhysicsLoopPeriod();
    void showClockStats();
};
