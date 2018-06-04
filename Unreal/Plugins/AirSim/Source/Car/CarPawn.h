#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/ConstructorHelpers.h"

#include "physics/Kinematics.hpp"
#include "CarPawnApi.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/common_utils/Signal.hpp"

#include "CarPawn.generated.h"

class UPhysicalMaterial;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class UInputComponent;
class UAudioComponent;

UCLASS(config = Game)
class ACarPawn : public AWheeledVehicle
{
    GENERATED_BODY()

public:
    typedef common_utils::Signal<UPrimitiveComponent*, AActor*, UPrimitiveComponent*, bool, FVector,
        FVector, FVector, const FHitResult&> CollisionSignal;

    ACarPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    //interface
    void initializeForBeginPlay(bool engine_sound);
    std::map<std::string, APIPCamera*> getCameras() const;
    CollisionSignal& getCollisionSignal()
    {
        return collision_signal_;
    }
    UWheeledVehicleMovementComponent* getVehicleMovementComponent() const;
    const CarPawnApi::CarControls& getKeyBoardControls() const
    {
        return keyboard_controls_;
    }

private:
    void updateHUDStrings();
    void setupVehicleMovementComponent();
    void updateInCarHUD();
    void updatePhysicsMaterial();

    void setupInputBindings();
    void onMoveForward(float Val);
    void onMoveRight(float Val);
    void onHandbrakePressed();
    void onHandbrakeReleased();
    void onFootBrake(float Val);
    void onReversePressed();
    void onReverseReleased();

private:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    UClass* pip_camera_class_;
    
    CollisionSignal collision_signal_;

    bool is_low_friction_;
    UPhysicalMaterial* slippery_mat_;
    UPhysicalMaterial* non_slippery_mat_;

    USceneComponent* camera_front_center_base_;
    USceneComponent* camera_front_left_base_;
    USceneComponent* camera_front_right_base_;
    USceneComponent* camera_driver_base_;
    USceneComponent* camera_back_center_base_;

    APIPCamera* camera_front_center_;
    APIPCamera* camera_front_left_;
    APIPCamera* camera_front_right_;
    APIPCamera* camera_driver_;
    APIPCamera* camera_back_center_;

    UTextRenderComponent* speed_text_render_;
    UTextRenderComponent* gear_text_render_;
    UAudioComponent* engine_sound_audio_;
    
    CarPawnApi::CarControls keyboard_controls_;

    FText last_speed_;
    FText last_gear_;
    FColor	last_gear_display_color_;
    FColor	last_gear_display_reverse_color_;
};
