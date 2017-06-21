#pragma once

#include <string>
#include "CameraDirector.h"
#include "GameFramework/Actor.h"
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
    virtual void Tick( float DeltaSeconds ) override;

    //additional overridable methods
    virtual void reset();
    virtual std::string getReport();
    virtual void startRecording();
    virtual void stopRecording();
    virtual bool isRecording();
    virtual bool isRecordUIVisible();
    virtual ECameraDirectorMode getInitialViewMode();

    FString getRecordingPath();

    std::ofstream record_file;
    std::string record_filename = "airsim_rec";    
protected:
    virtual void setupInputBindings();
    bool is_recording;
    bool is_record_ui_visible;
    ECameraDirectorMode initial_view_mode;
    int record_tick_count;
    bool enable_rpc;
    std::string api_server_address;
    std::string fpv_vehicle_name;

};
