#pragma once

#include "CoreMinimal.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "GameFramework/Actor.h"
#include "ParticleDefinitions.h"

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

    virtual void setTimeOfDay(bool is_enabled, const std::string& start_datetime, bool is_start_datetime_dst,
        float celestial_clock_speed, float update_interval_secs, bool move_sun);

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

protected: //must overrides
    typedef msr::airlib::AirSimSettings AirSimSettings;

    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const;
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const;
    virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const;
    virtual void initializeVehiclePawn(APawn* pawn);
    virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
        const PawnSimApi::Params& pawn_sim_api_params) const;
    virtual msr::airlib::VehicleApiBase* getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
        const PawnSimApi* sim_api) const;

protected: //optional overrides
    virtual void setupVehiclesAndCamera();
    virtual void setupInputBindings();
    //called when SimMode should handle clock speed setting
    virtual void setupClockSpeed();
    void initializeCameraDirector(const FTransform& camera_transform, float follow_distance);
    void checkVehicleReady(); //checks if vehicle is available to use
    virtual void updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter);

protected: //Utility methods for derived classes
    virtual const msr::airlib::AirSimSettings& getSettings() const;
    FRotator toFRotator(const AirSimSettings::Rotation& rotation, const FRotator& default_val);


protected:
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
    UPROPERTY() ADirectionalLight* sun_;
    FRotator default_sun_rotation_;
    TTimePoint tod_sim_clock_start_;             // sim start in local time
    TTimePoint tod_last_update_;
    TTimePoint tod_start_time_;                  // tod, configurable
    bool tod_enabled_;
    float tod_celestial_clock_speed_;
    float tod_update_interval_secs_;
    bool tod_move_sun_;

    std::unique_ptr<NedTransform> global_ned_transform_;
    std::unique_ptr<msr::airlib::WorldSimApiBase> world_sim_api_;
    std::unique_ptr<msr::airlib::ApiProvider> api_provider_;
    std::unique_ptr<msr::airlib::ApiServerBase> api_server_;
    msr::airlib::StateReporterWrapper debug_reporter_;

    std::vector<std::unique_ptr<msr::airlib::VehicleSimApiBase>> vehicle_sim_apis_;

    UPROPERTY()
        TArray<AActor*> spawned_actors_; //keep refs alive from Unreal GC

    bool lidar_checks_done_ = false; 
    bool lidar_draw_debug_points_ = false;

private:
    void setStencilIDs();
    void initializeTimeOfDay();
    void advanceTimeOfDay();
    void setSunRotation(FRotator rotation);
    void setupPhysicsLoopPeriod();
    void showClockStats();
    void drawLidarDebugPoints();
};
