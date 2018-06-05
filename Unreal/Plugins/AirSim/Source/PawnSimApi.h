#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include <vector>
#include <memory>
#include "UnrealImageCapture.h"
#include "common/Common.hpp"
#include "common/common_utils/Signal.hpp"
#include "common/CommonStructs.hpp"
#include "PIPCamera.h"
#include "physics/Kinematics.hpp"
#include "NedTransform.h"
#include "common/AirSimSettings.hpp"
#include "SimJoyStick/SimJoyStick.h"
#include "api/VehicleSimApiBase.hpp"


class PawnSimApi : public msr::airlib::VehicleSimApiBase {
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
    typedef common_utils::Signal<UPrimitiveComponent*, AActor*, UPrimitiveComponent*, bool, FVector,
        FVector, FVector, const FHitResult&> CollisionSignal;

public: //implementation of VehicleSimApiBase
    virtual void reset() override;
    virtual const UnrealImageCapture* getImageCapture() const override;
    virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& request) const override;
    virtual std::vector<uint8_t> getImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type) const override;
    virtual Pose getPose() const override;
    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual const msr::airlib::Kinematics::State* getGroundTruthKinematics() const override;
    virtual msr::airlib::CameraInfo getCameraInfo(const std::string& camera_name) const override;
    virtual void setCameraOrientation(const std::string& camera_name, const Quaternionr& orientation) override;
    virtual CollisionInfo getCollisionInfo() const override;
    virtual int getRemoteControlID() const override;
    virtual msr::airlib::RCData getRCData() const override;
    virtual std::string getVehicleName() const override
    {
        return vehicle_name_;
    }
    virtual void toggleTrace() override;

public: //Unreal specific methods
    PawnSimApi(APawn* pawn, const NedTransform& global_transform, CollisionSignal& collision_signal,
        const std::map<std::string, APIPCamera*>& cameras, UClass* pip_camera_class, UParticleSystem* collision_display_template);

    //returns one of the cameras attached to the pawn
    const APIPCamera* getCamera(const std::string& camera_name) const;
    APIPCamera* getCamera(const std::string& camera_name);
    int getCameraCount();

    //if enabled, this would show some flares
    void displayCollisionEffect(FVector hit_location, const FHitResult& hit);

    //return the attached pawn
    APawn* getPawn();

    //get/set pose
    //parameters in NED frame
    void setDebugPose(const Pose& debug_pose);

    FVector getUUPosition() const;
    FRotator getUUOrientation() const;

    const NedTransform& getNedTransform() const;

    void possess();
    void setRCForceFeedback(float rumble_strength, float auto_center);

    //allows setting ground truth from physics engine
    void setGroundTruthKinematics(const msr::airlib::Kinematics::State* kinematics);

private: //methods
    bool canTeleportWhileMove()  const;
    void allowPassthroughToggleInput();
    void detectUsbRc();
    void setupCamerasFromSettings(const std::map<std::string, APIPCamera*>& cameras);
    void createCamerasFromSettings();
    //on collision, pawns should update this
    void onCollision(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp,
        bool bSelfMoved, FVector HitLocation, FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit);

    //these methods are for future usage
    void plot(std::istream& s, FColor color, const Vector3r& offset);
    PawnSimApi::Pose toPose(const FVector& u_position, const FQuat& u_quat) const;

private: //vars
    typedef msr::airlib::AirSimSettings AirSimSettings;

    APawn* pawn_;
    //TODO: should below be TMap to keep refs alive?
    std::map<std::string, APIPCamera*> cameras_;
    std::string vehicle_name_;
    NedTransform ned_transform_;

    FVector ground_trace_end_;
    FVector ground_margin_;
    GeoPoint home_geo_point_;
    std::unique_ptr<UnrealImageCapture> image_capture_;
    const msr::airlib::Kinematics::State* kinematics_;
    std::string log_line_;

    mutable msr::airlib::RCData rc_data_;
    mutable SimJoyStick joystick_;
    mutable SimJoyStick::State joystick_state_;

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


    UPROPERTY() UClass* pip_camera_class_;
    UPROPERTY() UParticleSystem* collision_display_template_;

};
