#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"
#include "CarPawnApi.h"
#include "SimJoyStick/SimJoyStick.h"
#include "CarPawn.generated.h"


UCLASS(config = Game)
class ACarPawn : public AWheeledVehicle
{
    GENERATED_BODY()

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
    class APIPCamera* InternalCamera1;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class APIPCamera* InternalCamera2;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class APIPCamera* InternalCamera3;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class APIPCamera* InternalCamera4;
    UPROPERTY(Category = Camera, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class APIPCamera* InternalCamera5;

    /** Text component for the In-Car speed */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class UTextRenderComponent* InCarSpeed;

    /** Text component for the In-Car gear */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class UTextRenderComponent* InCarGear;

    /** Audio component for the engine sound */
    UPROPERTY(Category = Display, VisibleDefaultsOnly, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
    class UAudioComponent* EngineSoundComponent;

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

    // Begin Actor interface
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    class VehiclePawnWrapper* getVehiclePawnWrapper();
    void initializeForBeginPlay(bool engine_sound);

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
    void onHandbrakePressed();
    /** Handle handbrake released */
    void onHandbrakeReleased();
    /** Handle pressiong footbrake */
    void FootBrake(float Val);
    /** Handle Reverse pressed */
    void onReversePressed();
    /** Handle Reverse released */
    void onReverseReleased();
    /** Handle Handbrake pressed */

    /** Setup the strings used on the hud */
    void updateInCarHUD();

    /** update the physics material used by the vehicle mesh */
    void updatePhysicsMaterial();

private:
    /** update the gear and speed strings */
    void updateHUDStrings();
    void updateKinematics(float delta);
    void updateCarControls();
    void updateForceFeedback();

    std::string getLogString();
    void setupVehicleMovementComponent();
    msr::airlib::CarApiBase* getApi() const;

private:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    UClass* pip_camera_class_;

    std::unique_ptr<VehiclePawnWrapper> wrapper_;
    msr::airlib::Kinematics::State kinematics_;

    CarPawnApi::CarControls keyboard_controls_;
    CarPawnApi::CarControls joystick_controls_;
    CarPawnApi::CarControls current_controls_;

    SimJoyStick joystick_;
    SimJoyStick::State joystick_state_;


    /* Are we on a 'slippery' surface */
    bool is_low_friction_;
    /** Slippery Material instance */
    class UPhysicalMaterial* slippery_mat_;
    /** Non Slippery Material instance */
    class UPhysicalMaterial* non_slippery_mat_;
};
