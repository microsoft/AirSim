#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include <vector>
#include <memory>
#include "UnrealImageCapture.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "PIPCamera.h"
#include "physics/Kinematics.hpp"
#include "NedTransform.h"
#include "common/AirSimSettings.hpp"
#include "api/VehicleSimApiBase.hpp"


class VehicleSimApi : public msr::airlib::VehicleSimApiBase {
public: //types
    typedef msr::airlib::GeoPoint GeoPoint;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::CollisionInfo CollisionInfo;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

    struct WrapperConfig {
        bool is_fpv_vehicle;
        bool enable_collisions;
        bool enable_passthrough_on_collisions;
        bool enable_trace;

        WrapperConfig() :
            is_fpv_vehicle(false),
            enable_collisions(true),
            enable_passthrough_on_collisions(false),
            enable_trace(false)
        {
        }
    }; 

public: //implementation of VehicleSimApiBase
    virtual void reset() override;
    virtual const UnrealImageCapture* getImageCapture() const override;
    virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& request) const override;
    virtual std::vector<uint8_t> getImage(uint8_t camera_id, ImageCaptureBase::ImageType image_type) const override;
    virtual Pose getPose() const override;
    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual const msr::airlib::Kinematics::State* getGroundTruthKinematics() const override;
    virtual msr::airlib::CameraInfo getCameraInfo(int camera_id) const override;
    virtual void setCameraOrientation(int camera_id, const Quaternionr& orientation) override;
    virtual CollisionInfo getCollisionInfo() const override;
    virtual int getRemoteControlID() const override;


public: //Unreal specific methods
    VehicleSimApi();

    //this class should be constructed on BeginPlay and initialize method should be the first call
    void initialize(APawn* pawn, const std::vector<APIPCamera*>& cameras, const std::string& vehicle_name, 
        const WrapperConfig& config = WrapperConfig());

    //on collision, pawns should update this
    void onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, 
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit);

    //returns one of the cameras attached to the pawn
    const APIPCamera* getCamera(int index = 0) const;
    APIPCamera* getCamera(int index = 0);
    int getCameraCount();

    //if enabled, this would show some flares
    void displayCollisionEffect(FVector hit_location, const FHitResult& hit);

    //return the attached pawn
    APawn* getPawn();

    //get/set pose
    //parameters in NED frame
    void setDebugPose(const Pose& debug_pose);

    void setLogLine(std::string line);
    std::string getLogLine();

    WrapperConfig& getConfig();
    const WrapperConfig& getConfig() const;

    const VehicleSetting* getVehicleSetting() const;

    FVector getUUPosition() const;
    FRotator getUUOrientation() const;

    const NedTransform& getNedTransform() const;

    void possess();

    void toggleTrace();

protected:
    UPROPERTY(VisibleAnywhere)
        UParticleSystem* collision_display_template;

    //allows setting ground truth from physics engine
    void setGroundTruthKinematics(const msr::airlib::Kinematics::State* kinematics);

private: //methods
    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();
    void setupCamerasFromSettings();

    //these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    VehicleSimApi::Pose toPose(const FVector& u_position, const FQuat& u_quat) const;

private: //vars
    FVector ground_trace_end_;
    FVector ground_margin_;
    GeoPoint home_geo_point_;
    APawn* pawn_;
    std::vector<APIPCamera*> cameras_;
    std::unique_ptr<UnrealImageCapture> image_capture_;
    const msr::airlib::Kinematics::State* kinematics_;
    std::string log_line_;
    WrapperConfig config_;
    NedTransform ned_transform_;
    std::string vehicle_name_;
    const VehicleSetting* vehicle_setting_;

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
