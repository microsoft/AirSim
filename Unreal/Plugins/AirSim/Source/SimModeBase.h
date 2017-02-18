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

    UFUNCTION(BlueprintCallable, Category = "Debugging")
    FString getReportBP();

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

    std::string record_filename = "c:\\temp\\airsim_rec.txt";
    std::string record_folder_path = "c:\\temp\\airsim\\img_";
  
protected:
    virtual void setupInputBindings();
    bool is_recording;
    std::ofstream record_file;
    int record_tick_count;
};
