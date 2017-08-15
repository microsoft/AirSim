#pragma once

#include "CoreMinimal.h"
#include "VehicleConnectorBase.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "PIPCamera.h"
#include "controllers/Settings.hpp"
#include "GameFramework/Pawn.h"
#include "VehiclePawnBase.generated.h"


UCLASS()
class AIRSIM_API AVehiclePawnBase : public APawn
{
    GENERATED_BODY()

public: //types
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::CollisionInfo CollisionInfo;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;

public: //modifiable properties
    //Vehicle config settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VehicleConfig")
    bool IsFpvVehicle = false; 
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VehicleConfig")
    FString VehicleConfigName = ""; //use the default config name

    //collisons settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collisons")
    bool EnableCollisons = true; 
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Collisons")
    bool EnablePassthroughOnCollisons = false; 

    //GPS settings
    //defaults are Microsoft Building 99, WA, USA
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPS")
    float HomeLatitude = 47.641468;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPS")
    float HomeLongitude = -122.140165;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GPS")
    float HomeAltitude = 122;

    //trace settings
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debugging")
    bool EnableTrace = false; 

    UFUNCTION(BlueprintCallable, Category = "Debugging")
    void toggleTrace();

public: //interface
    AVehiclePawnBase();

    //overridden from pawn
    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, 
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface - derived class should override these as needed
    virtual void initialize(); //called at contruction time
    virtual void initializeForBeginPlay();
    virtual void reset();
    UFUNCTION(BlueprintCallable, Category = "Camera")
    virtual APIPCamera* getCamera(int index = 0);
    virtual int getCameraCount();
    virtual void displayCollisonEffect(FVector hit_location, const FHitResult& hit);

    //get/set pose
    //parameters in NED frame
    Pose getPose() const;
    void setPose(const Pose& pose, const Pose& debug_pose);
    FVector getPosition() const;
    FRotator getOrientation() const;

    //get configurations
    real_T getMinZOverGround() const;
    const GeoPoint& getHomePoint() const;
    const CollisionInfo& getCollisonInfo() const;

protected:
    UPROPERTY(VisibleAnywhere)
        UParticleSystem* collison_display_template;

    void setupCamerasFromSettings();

private: //methods
    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();

    //these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    void createCaptureSettings(const msr::airlib::Settings& settings, APIPCamera::CaptureSettings& capture_settings);


private: //vars
    FVector ground_trace_end;
    FVector ground_margin;
    GeoPoint home_point;

    struct State {
        FVector start_location;
        FRotator start_rotation;
        FVector last_position;
        FVector last_debug_position;
        FVector current_position;
        FVector current_debug_position;
        FVector debug_position_offset;        
        bool tracing_enabled;
        bool collisons_enabled;
        bool passthrough_enabled;
        bool was_last_move_teleport;
        CollisionInfo collison_info;

        FVector mesh_origin;
        FVector mesh_bounds;
        FVector ground_offset;
        FVector transformation_offset;
    };
    
    State state_, initial_state_;
};
