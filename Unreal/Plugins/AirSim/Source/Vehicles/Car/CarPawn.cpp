#include "CarPawn.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "WheeledVehicleMovementComponent4W.h"

#include "CarWheelFront.h"
#include "CarWheelRear.h"
#include "AirBlueprintLib.h"
#include <vector>
#include "common/common_utils/Utils.hpp"
#include "common/ClockFactory.hpp"

#define LOCTEXT_NAMESPACE "VehiclePawn"

ACarPawn::ACarPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

    const auto& car_mesh_paths = AirSimSettings::singleton().pawn_paths["DefaultCar"];
    auto slippery_mat = Cast<UPhysicalMaterial>(
        UAirBlueprintLib::LoadObject(car_mesh_paths.slippery_mat));
    auto non_slippery_mat = Cast<UPhysicalMaterial>(
        UAirBlueprintLib::LoadObject(car_mesh_paths.non_slippery_mat));
    if (slippery_mat)
        slippery_mat_ = slippery_mat;
    else
        UAirBlueprintLib::LogMessageString("Failed to load Slippery physics material", "", LogDebugLevel::Failure);
    if (non_slippery_mat)
        non_slippery_mat_ = non_slippery_mat;
    else
        UAirBlueprintLib::LogMessageString("Failed to load NonSlippery physics material", "", LogDebugLevel::Failure);

    setupVehicleMovementComponent();

    // Create In-Car camera component
    camera_front_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_center_base_"));
    camera_front_center_base_->SetRelativeLocation(FVector(200, 0, 100)); //center
    camera_front_center_base_->SetupAttachment(GetMesh());
    camera_front_left_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_left_base_"));
    camera_front_left_base_->SetRelativeLocation(FVector(200, -12.5, 100)); //left
    camera_front_left_base_->SetupAttachment(GetMesh());
    camera_front_right_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_front_right_base_"));
    camera_front_right_base_->SetRelativeLocation(FVector(200, 12.5, 100)); //right
    camera_front_right_base_->SetupAttachment(GetMesh());
    camera_driver_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_driver_base_"));
    camera_driver_base_->SetRelativeLocation(FVector(0, -25, 125)); //driver
    camera_driver_base_->SetupAttachment(GetMesh());
    camera_back_center_base_ = CreateDefaultSubobject<USceneComponent>(TEXT("camera_back_center_base_"));
    camera_back_center_base_->SetRelativeLocation(FVector(-200, 0, 100)); //rear
    camera_back_center_base_->SetupAttachment(GetMesh());

    // In car HUD
    // Create text render component for in car speed display
    speed_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarSpeed"));
    speed_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    speed_text_render_->SetRelativeLocation(FVector(35.0f, -6.0f, 20.0f));
    speed_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    speed_text_render_->SetupAttachment(GetMesh());
    speed_text_render_->SetVisibility(true);

    // Create text render component for in car gear display
    gear_text_render_ = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
    gear_text_render_->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    gear_text_render_->SetRelativeLocation(FVector(35.0f, 5.0f, 20.0f));
    gear_text_render_->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    gear_text_render_->SetupAttachment(GetMesh());
    gear_text_render_->SetVisibility(true);

    // Setup the audio component and allocate it a sound cue
    ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/AirSim/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
    engine_sound_audio_ = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
    engine_sound_audio_->SetSound(SoundCue.Object);
    engine_sound_audio_->SetupAttachment(GetMesh());

    // Colors for the in-car gear display. One for normal one for reverse
    last_gear_display_reverse_color_ = FColor(255, 0, 0, 255);
    last_gear_display_color_ = FColor(255, 255, 255, 255);

    is_low_friction_ = false;
}

void ACarPawn::setupVehicleMovementComponent()
{
    UWheeledVehicleMovementComponent4W* movement = CastChecked<UWheeledVehicleMovementComponent4W>(getVehicleMovementComponent());
    check(movement->WheelSetups.Num() == 4);

    // Wheels/Tires
    // Setup the wheels
    movement->WheelSetups[0].WheelClass = UCarWheelFront::StaticClass();
    movement->WheelSetups[0].BoneName = FName("PhysWheel_FL");
    movement->WheelSetups[0].AdditionalOffset = FVector(0.f, -8.f, 0.f);

    movement->WheelSetups[1].WheelClass = UCarWheelFront::StaticClass();
    movement->WheelSetups[1].BoneName = FName("PhysWheel_FR");
    movement->WheelSetups[1].AdditionalOffset = FVector(0.f, 8.f, 0.f);

    movement->WheelSetups[2].WheelClass = UCarWheelRear::StaticClass();
    movement->WheelSetups[2].BoneName = FName("PhysWheel_BL");
    movement->WheelSetups[2].AdditionalOffset = FVector(0.f, -8.f, 0.f);

    movement->WheelSetups[3].WheelClass = UCarWheelRear::StaticClass();
    movement->WheelSetups[3].BoneName = FName("PhysWheel_BR");
    movement->WheelSetups[3].AdditionalOffset = FVector(0.f, 8.f, 0.f);

    // Adjust the tire loading
    movement->MinNormalizedTireLoad = 0.0f;
    movement->MinNormalizedTireLoadFiltered = 0.2308f;
    movement->MaxNormalizedTireLoad = 2.0f;
    movement->MaxNormalizedTireLoadFiltered = 2.0f;

    // Engine
    // Torque setup
    movement->EngineSetup.MaxRPM = 5700.0f;
    movement->EngineSetup.TorqueCurve.GetRichCurve()->Reset();
    movement->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(0.0f, 400.0f);
    movement->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(1890.0f, 500.0f);
    movement->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(5730.0f, 400.0f);

    // Adjust the steering
    movement->SteeringCurve.GetRichCurve()->Reset();
    movement->SteeringCurve.GetRichCurve()->AddKey(0.0f, 1.0f);
    movement->SteeringCurve.GetRichCurve()->AddKey(40.0f, 0.7f);
    movement->SteeringCurve.GetRichCurve()->AddKey(120.0f, 0.6f);

    // Transmission
    // We want 4wd
    movement->DifferentialSetup.DifferentialType = EVehicleDifferential4W::LimitedSlip_4W;

    // Drive the front wheels a little more than the rear
    movement->DifferentialSetup.FrontRearSplit = 0.65;

    // Automatic gearbox
    movement->TransmissionSetup.bUseGearAutoBox = true;
    movement->TransmissionSetup.GearSwitchTime = 0.15f;
    movement->TransmissionSetup.GearAutoBoxLatency = 1.0f;

    // Disable reverse as brake, this is needed for SetBreakInput() to take effect
    movement->bReverseAsBrake = false;

    // Physics settings
    // Adjust the center of mass - the buggy is quite low
    UPrimitiveComponent* primitive = Cast<UPrimitiveComponent>(movement->UpdatedComponent);
    if (primitive) {
        primitive->BodyInstance.COMNudge = FVector(8.0f, 0.0f, 0.0f);
    }

    // Set the inertia scale. This controls how the mass of the vehicle is distributed.
    movement->InertiaTensorScale = FVector(1.0f, 1.333f, 1.2f);
    movement->bDeprecatedSpringOffsetMode = true;
}

void ACarPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                         FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
}

UWheeledVehicleMovementComponent* ACarPawn::getVehicleMovementComponent() const
{
    return GetVehicleMovement();
}

void ACarPawn::initializeForBeginPlay(bool engine_sound)
{
    if (engine_sound)
        engine_sound_audio_->Activate();
    else
        engine_sound_audio_->Deactivate();

    //put camera little bit above vehicle
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_center"));
    camera_front_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_center_->AttachToComponent(camera_front_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_left"));
    camera_front_left_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_left_->AttachToComponent(camera_front_left_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_front_right"));
    camera_front_right_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_front_right_->AttachToComponent(camera_front_right_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_driver"));
    camera_driver_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    camera_driver_->AttachToComponent(camera_driver_base_, FAttachmentTransformRules::KeepRelativeTransform);

    camera_spawn_params.Name = FName(*(this->GetName() + "_camera_back_center"));
    camera_back_center_ = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_,
                                                                   FTransform(FRotator(0, -180, 0), FVector::ZeroVector),
                                                                   camera_spawn_params);
    camera_back_center_->AttachToComponent(camera_back_center_base_, FAttachmentTransformRules::KeepRelativeTransform);

    setupInputBindings();
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ACarPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("fpv", camera_driver_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_driver_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);

    return cameras;
}

void ACarPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_center_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_right_ = nullptr;
    camera_driver_ = nullptr;
    camera_back_center_ = nullptr;

    camera_front_center_base_ = nullptr;
    camera_front_left_base_ = nullptr;
    camera_front_right_base_ = nullptr;
    camera_driver_base_ = nullptr;
    camera_back_center_base_ = nullptr;
}

void ACarPawn::Tick(float Delta)
{
    Super::Tick(Delta);

    // update physics material
    updatePhysicsMaterial();

    // Update the strings used in the HUD (in-car and on-screen)
    updateHUDStrings();

    // Set the string in the in-car HUD
    updateInCarHUD();

    // Pass the engine RPM to the sound component
    float RPMToAudioScale = 2500.0f / GetVehicleMovement()->GetEngineMaxRotationSpeed();
    engine_sound_audio_->SetFloatParameter(FName("RPM"), GetVehicleMovement()->GetEngineRotationSpeed() * RPMToAudioScale);

    pawn_events_.getPawnTickSignal().emit(Delta);
}

void ACarPawn::BeginPlay()
{
    Super::BeginPlay();

    // Start an engine sound playing
    engine_sound_audio_->Play();
}

void ACarPawn::updateHUDStrings()
{

    float speed_unit_factor = AirSimSettings::singleton().speed_unit_factor;
    FText speed_unit_label = FText::FromString(FString(AirSimSettings::singleton().speed_unit_label.c_str()));
    float vel = FMath::Abs(GetVehicleMovement()->GetForwardSpeed() / 100); //cm/s -> m/s
    float vel_rounded = FMath::FloorToInt(vel * 10 * speed_unit_factor) / 10.0f;
    int32 Gear = GetVehicleMovement()->GetCurrentGear();

    // Using FText because this is display text that should be localizable
    last_speed_ = FText::Format(LOCTEXT("SpeedFormat", "{0} {1}"), FText::AsNumber(vel_rounded), speed_unit_label);

    if (GetVehicleMovement()->GetCurrentGear() < 0) {
        last_gear_ = FText(LOCTEXT("ReverseGear", "R"));
    }
    else {
        last_gear_ = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
    }

    UAirBlueprintLib::LogMessage(TEXT("Speed: "), last_speed_.ToString(), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessage(TEXT("Gear: "), last_gear_.ToString(), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessage(TEXT("RPM: "), FText::AsNumber(GetVehicleMovement()->GetEngineRotationSpeed()).ToString(), LogDebugLevel::Informational);
}

void ACarPawn::updateInCarHUD()
{
    APlayerController* PlayerController = Cast<APlayerController>(GetController());
    if ((PlayerController != nullptr) && (speed_text_render_ != nullptr) && (gear_text_render_ != nullptr)) {
        // Setup the text render component strings
        speed_text_render_->SetText(last_speed_);
        gear_text_render_->SetText(last_gear_);

        if (GetVehicleMovement()->GetCurrentGear() >= 0) {
            gear_text_render_->SetTextRenderColor(last_gear_display_color_);
        }
        else {
            gear_text_render_->SetTextRenderColor(last_gear_display_reverse_color_);
        }
    }
}

void ACarPawn::updatePhysicsMaterial()
{
    if (GetActorUpVector().Z < 0) {
        if (is_low_friction_ == true) {
            GetMesh()->SetPhysMaterialOverride(non_slippery_mat_);
            is_low_friction_ = false;
        }
        else {
            GetMesh()->SetPhysMaterialOverride(slippery_mat_);
            is_low_friction_ = true;
        }
    }
}

/******************* Keyboard bindings*******************/
//This method must be in pawn because Unreal doesn't allow key bindings to non UObject pointers
void ACarPawn::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Up, 1), this, this, &ACarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Down, -1), this, this, &ACarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Right, 0.5), this, this, &ACarPawn::onMoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Left, -0.5), this, this, &ACarPawn::onMoveRight);

    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::onHandbrakePressed, true);
    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::onHandbrakeReleased, false);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::SpaceBar, 1), this, this, &ACarPawn::onFootBrake);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Gamepad_LeftX, 1), this, this, &ACarPawn::onMoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Gamepad_RightTriggerAxis, 1), this, this, &ACarPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::Gamepad_LeftTriggerAxis, 1), this, this, &ACarPawn::onFootBrake);

    //below is not needed
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::onReversePressed, true);
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::onReverseReleased, false);
}

void ACarPawn::onMoveForward(float Val)
{
    if (Val < 0)
        onReversePressed();
    else
        onReverseReleased();

    keyboard_controls_.throttle = Val;
}

void ACarPawn::onMoveRight(float Val)
{
    keyboard_controls_.steering = Val;
}

void ACarPawn::onHandbrakePressed()
{
    keyboard_controls_.handbrake = true;
}

void ACarPawn::onHandbrakeReleased()
{
    keyboard_controls_.handbrake = false;
}

void ACarPawn::onFootBrake(float Val)
{
    keyboard_controls_.brake = Val;
}

void ACarPawn::onReversePressed()
{
    if (keyboard_controls_.manual_gear >= 0) {
        keyboard_controls_.is_manual_gear = true;
        keyboard_controls_.manual_gear = -1;
        keyboard_controls_.gear_immediate = true;
    }
}

void ACarPawn::onReverseReleased()
{
    if (keyboard_controls_.manual_gear < 0) {
        keyboard_controls_.is_manual_gear = false;
        keyboard_controls_.manual_gear = 0;
        keyboard_controls_.gear_immediate = true;
    }
}

#undef LOCTEXT_NAMESPACE
