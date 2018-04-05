#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "vehicles/car/api/CarRpcLibServer.hpp"
#include "physics/Kinematics.hpp"
#include "CarPawnApi.h"
#include "SimJoyStick/SimJoyStick.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
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

    /** Camera component that will be our viewpoint */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UCameraComponent* Camera;

    /** SCene component for the In-Car view origin */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase1;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase2;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase3;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase4;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class USceneComponent* InternalCameraBase5;

    /** Camera component for the In-Car view */
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    APIPCamera* InternalCamera1;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    APIPCamera* InternalCamera2;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    APIPCamera* InternalCamera3;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    APIPCamera* InternalCamera4;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    APIPCamera* InternalCamera5;

    /** Text component for the In-Car speed */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UTextRenderComponent* InCarSpeed;

    /** Text component for the In-Car gear */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UTextRenderComponent* InCarGear;

    /** Audio component for the engine sound */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    UAudioComponent* EngineSoundComponent;


    /** Whether to load the default meshes */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    bool UseDefaultMesh = false;

public:
    ACarPawn();

    /** The current speed as a string eg 10 km/h */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FText SpeedDisplayString;

    /** The current gear as a string (R,N, 1,2 etc) */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FText GearDisplayString;

    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    /** The color of the incar gear text in forward gears */
    FColor	GearDisplayColor;

    /** The color of the incar gear text when in reverse */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly)
    FColor	GearDisplayReverseColor;

    void setupInputBindings();

    void reset(bool disable_api_control = true);

    // Begin Actor interface
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    VehiclePawnWrapper* getVehiclePawnWrapper();
    void initializeForBeginPlay(bool enable_rpc, const std::string& api_server_address, bool engine_sound);

    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
        FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;
protected:
    virtual void BeginPlay() override;

public:
    // End Actor interface

    /** Handle pressing forwards */
    void MoveForward(float Val);
    /** Handle pressing right */
    void MoveRight(float Val);
    /** Handle handbrake pressed */
    void OnHandbrakePressed();
    /** Handle handbrake released */
    void OnHandbrakeReleased();
    /** Handle pressiong footbrake */
    void FootBrake(float Val);
    /** Handle Reverse pressed */
    void OnReversePressed();
    /** Handle Reverse released */
    void OnReverseReleased();
    /** Handle Handbrake pressed */

    /** Setup the strings used on the hud */
    void UpdateInCarHUD();

    /** Update the physics material used by the vehicle mesh */
    void UpdatePhysicsMaterial();

    static const FName LookUpBinding;
    static const FName LookRightBinding;
    static const FName EngineAudioRPM;

private:
    /** Update the gear and speed strings */
    void UpdateHUDStrings();
    void startApiServer(bool enable_rpc, const std::string& api_server_address);
    void stopApiServer();
    bool isApiServerStarted();
    void updateKinematics(float delta);
    void updateCarControls();

    std::string getLogString();

    /* Are we on a 'slippery' surface */
    bool bIsLowFriction;
    /** Slippery Material instance */
    UPhysicalMaterial* SlipperyMaterial;
    /** Non Slippery Material instance */
    UPhysicalMaterial* NonSlipperyMaterial;

public:
    /** Returns InCarSpeed subobject **/
    FORCEINLINE UTextRenderComponent* GetInCarSpeed() const { return InCarSpeed; }
    /** Returns InCarGear subobject **/
    FORCEINLINE UTextRenderComponent* GetInCarGear() const { return InCarGear; }
    /** Returns EngineSoundComponent subobject **/
    FORCEINLINE UAudioComponent* GetEngineSoundComponent() const { return EngineSoundComponent; }

private:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    struct MeshContructionHelpers {
        USkeletalMesh* skeleton;
        UBlueprint* bp;
        UPhysicalMaterial* slippery_mat;
        UPhysicalMaterial* non_slippery_mat;

        MeshContructionHelpers(const msr::airlib::AirSimSettings::CarMeshPaths& paths)
        {
            skeleton = Cast<USkeletalMesh>(UAirBlueprintLib::LoadObject(paths.skeletal));
            bp = Cast<UBlueprint>(UAirBlueprintLib::LoadObject(paths.bp));
            slippery_mat = Cast<UPhysicalMaterial>(UAirBlueprintLib::LoadObject(paths.slippery_mat));
            non_slippery_mat = Cast<UPhysicalMaterial>(UAirBlueprintLib::LoadObject(paths.non_slippery_mat));
        }

    };


    UClass* pip_camera_class_;

    std::unique_ptr<msr::airlib::CarRpcLibServer> rpclib_server_;
    std::unique_ptr<msr::airlib::CarApiBase> api_;
    std::unique_ptr<VehiclePawnWrapper> wrapper_;
    msr::airlib::Kinematics::State kinematics_;

    CarPawnApi::CarControls keyboard_controls_;
    CarPawnApi::CarControls joystick_controls_;
    CarPawnApi::CarControls current_controls_;

    SimJoyStick joystick_;
    SimJoyStick::State joystick_state_;
};
