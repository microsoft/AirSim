#pragma once

#include "CoreMinimal.h"
#include <vector>
#include <memory>
#include "UnrealImageCapture.h"
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
        bool is_fpv_vehicle; 
        std::string vehicle_config_name;
        bool enable_collisions; 
        bool enable_passthrough_on_collisions; 
        float home_lattitude;
        float home_longitude;
        float home_altitude;
        bool enable_trace;

        WrapperConfig() :
            is_fpv_vehicle(false),
            vehicle_config_name(""), //use the default config name
            enable_collisions(true),
            enable_passthrough_on_collisions(false),
            home_lattitude(47.641468),
            home_longitude(-122.140165),
            home_altitude(122),
            enable_trace(false)
        {
        }
    };

    void toggleTrace();

public: //interface
    VehiclePawnWrapper();
    void initialize(APawn* pawn, const std::vector<APIPCamera*>& cameras, const WrapperConfig& config = WrapperConfig());

    void reset();
    void onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, 
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit);

    APIPCamera* getCamera(int index = 0);
    UnrealImageCapture* getImageCapture();
    int getCameraCount();
    void displayCollisionEffect(FVector hit_location, const FHitResult& hit);
    APawn* getPawn();

    //get/set pose
    //parameters in NED frame
    Pose getPose() const;
    void setPose(const Pose& pose, bool ignore_collision);
    void setDebugPose(const Pose& debug_pose);
    FVector getPosition() const;
    FRotator getOrientation() const;

    void setKinematics(const msr::airlib::Kinematics::State* kinematics);
    const msr::airlib::Kinematics::State* getTrueKinematics();

    const GeoPoint& getHomePoint() const;
    const CollisionInfo& getCollisionInfo() const;

    void setLogLine(std::string line);
    std::string getLogLine();

    void printLogMessage(const std::string& message, const std::string& message_param = "", unsigned char severity = 0);

    WrapperConfig& getConfig();
    const WrapperConfig& getConfig() const;

    static VehiclePawnWrapper::Pose toPose(const FVector& u_position, const FQuat& u_quat);
    msr::airlib::Pose getActorPose(std::string actor_name);


protected:
    UPROPERTY(VisibleAnywhere)
        UParticleSystem* collision_display_template;


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
    std::unique_ptr<UnrealImageCapture> image_capture_;
    const msr::airlib::Kinematics::State* kinematics_;
    std::string log_line_;
    WrapperConfig config_;

    struct State {
        FVector start_location;
        FRotator start_rotation;
        FVector last_position;
        FVector last_debug_position;
        FVector current_position;
        FVector current_debug_position;
        FVector debug_position_offset;        
        bool tracing_enabled;
        bool collisions_enabled;
        bool passthrough_enabled;
        bool was_last_move_teleport;
        CollisionInfo collision_info;

        FVector mesh_origin;
        FVector mesh_bounds;
        FVector ground_offset;
        FVector transformation_offset;
    };
    
    State state_, initial_state_;
};
