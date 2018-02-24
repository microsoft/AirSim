#pragma once

#include "CoreMinimal.h"
#include <string>
#include "CameraDirector.h"
#include "GameFramework/Actor.h"
#include "ManualPoseController.h"
#include "VehiclePawnWrapper.h"
#include "common/AirSimSettings.hpp"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
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
    virtual bool isRecording();
    virtual bool isRecordUIVisible();
    virtual ECameraDirectorMode getInitialViewMode();

    //must be implemented by derived class
    //can't use pure virtual because of restriction with Unreal
    virtual VehiclePawnWrapper* getFpvVehiclePawnWrapper();


protected:
    typedef msr::airlib::AirSimSettings AirSimSettings;
    virtual void setupInputBindings();
    virtual const AirSimSettings& getSettings() const;
    long long getPhysicsLoopPeriod();

protected: //settings
    int record_tick_count;
    static const char kUsageScenarioComputerVision[];


private:
    typedef common_utils::Utils Utils;
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef msr::airlib::TTimePoint TTimePoint;

private:
    UClass* sky_sphere_class_;
    UPROPERTY() AActor* sky_sphere_;
    UPROPERTY() ADirectionalLight* sun_;;
    TTimePoint tod_sim_clock_start_;
    TTimePoint tod_last_update_;
    std::time_t tod_start_time_;

private:
    void setStencilIDs();
    void setupTimeOfDay();
    void setupClock();
    void advanceTimeOfDay();
};
