#pragma once

#include "VehicleConnectorBase.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "PIPCamera.h"
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
    //overridden from pawn
    virtual void PostInitializeComponents() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface - derived class should override these as needed
    virtual void initialize();
    virtual void reset();
    UFUNCTION(BlueprintCallable, Category = "Camera")
    virtual APIPCamera* getFpvCamera();

    //get/set pose
    //parameters in NED frame
    Pose getPose() const;
    void setPose(const Pose& pose);
    void setPose(const Vector3r& position, const Quaternionr& orientation);
    //parameters in NEU frame
    void setPose(const FVector& position, const FQuat& orientation, bool enable_teleport);
    FVector getPosition() const;
    FRotator getOrientation() const;

    //get configurations
    real_T getMinZOverGround() const;
    const GeoPoint& getHomePoint() const;
    const CollisionInfo& getCollisonInfo() const;

    //utility methods
    static FVector toFVector(const Vector3r& vec, float scale, bool convert_from_ned);
    static FQuat toFQuat(const Quaternionr& q, bool convert_from_ned);
    static Vector3r toVector3r(const FVector& vec, float scale, bool convert_to_ned);
    static Quaternionr toQuaternionr(const FQuat& q, bool convert_to_ned);
    Vector3r toNedMeters(const FVector& position) const;
    FVector  toNeuUU(const Vector3r& position) const;

private: //methods
    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();

private: //vars
    FVector ground_trace_end;
    FVector ground_margin;
    float world_to_meters;
    GeoPoint home_point;

    struct State {
        FVector start_location;
        FRotator start_rotation;
        FVector last_position;
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
