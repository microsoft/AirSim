#pragma once

#include "CoreMinimal.h"
#include <vector>
#include <memory>
#include "VehicleCameraConnector.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "PIPCamera.h"
#include "controllers/Settings.hpp"
#include "physics/Kinematics.hpp"
#include "GameFramework/Pawn.h"

class VehiclePawnWrapper
{
public: //types
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::CollisionInfo CollisionInfo;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;

public:
    struct WrapperConfig {
        bool is_fpv_vehicle = false; 
        FString vehicle_config_name = ""; //use the default config name
        bool enable_collisions = true; 
        bool enable_passthrough_on_collisions = false; 
        float home_lattitude = 47.641468;
        float home_longitude = -122.140165;
        float home_altitude = 122;
        bool enable_trace = false;
    } config;

    void toggleTrace();

public: //interface
    VehiclePawnWrapper();
    void initialize(APawn* pawn, const std::vector<APIPCamera*>& cameras);

    void reset();
    void onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, 
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit);

    APIPCamera* getCamera(int index = 0);
    VehicleCameraConnector* getCameraConnector(int index = 0);
    int getCameraCount();
    void displayCollisonEffect(FVector hit_location, const FHitResult& hit);
    APawn* getPawn();

    //get/set pose
    //parameters in NED frame
    Pose getPose() const;
    void setPose(const Pose& pose, bool ignore_collison);
    void setDebugPose(const Pose& debug_pose);
    FVector getPosition() const;
    FRotator getOrientation() const;
    msr::airlib::Kinematics* getKinematics() const;

    const GeoPoint& getHomePoint() const;
    const CollisionInfo& getCollisonInfo() const;

protected:
    UPROPERTY(VisibleAnywhere)
        UParticleSystem* collison_display_template;


private: //methods
    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();
    void setupCamerasFromSettings();

    //these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    void createCaptureSettings(const msr::airlib::Settings& settings, APIPCamera::CaptureSettings& capture_settings);


private: //vars
    FVector ground_trace_end_;
    FVector ground_margin_;
    GeoPoint home_point_;
    APawn* pawn_;
    std::vector<APIPCamera*> cameras_;
    std::vector<std::unique_ptr<VehicleCameraConnector>> camera_connectors_;

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
