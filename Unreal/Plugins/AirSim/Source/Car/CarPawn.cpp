// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#include "CarPawn.h"
#include "CarWheelFront.h"
#include "CarWheelRear.h"
#include "Components/SkeletalMeshComponent.h"
#include "common/common_utils/Utils.hpp"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "AirBlueprintLib.h"
#include "NedTransform.h"
#include "PIPCamera.h"
#include <vector>
#include "UObject/ConstructorHelpers.h"

// Needed for VR Headset
#if HMD_MODULE_INCLUDED
#include "IHeadMountedDisplay.h"
#endif // HMD_MODULE_INCLUDED

const FName ACarPawn::LookUpBinding("LookUp");
const FName ACarPawn::LookRightBinding("LookRight");
const FName ACarPawn::EngineAudioRPM("RPM");

#define LOCTEXT_NAMESPACE "VehiclePawn"

class ACarPawn::CarController : public msr::airlib::CarApiBase {
public:
    typedef msr::airlib::CarApiBase CarApiBase;
    typedef msr::airlib::VehicleCameraBase VehicleCameraBase;

    CarController(ACarPawn* car_pawn)
        : car_pawn_(car_pawn)
    {
    }

    virtual std::vector<VehicleCameraBase::ImageResponse> simGetImages(
        const std::vector<VehicleCameraBase::ImageRequest>& request) override
    {
        std::vector<VehicleCameraBase::ImageResponse> response;

        for (const auto& item : request) {
            VehicleCameraBase* camera = car_pawn_->getVehiclePawnWrapper()->getCameraConnector(item.camera_id);
            const auto& item_response = camera->getImage(item.image_type, item.pixels_as_float, item.compress);
            response.push_back(item_response);
        }

        return response;
    }

    virtual std::vector<uint8_t> simGetImage(uint8_t camera_id, VehicleCameraBase::ImageType image_type) override
    {
        std::vector<VehicleCameraBase::ImageRequest> request = { VehicleCameraBase::ImageRequest(camera_id, image_type) };
        const std::vector<VehicleCameraBase::ImageResponse>& response = simGetImages(request);
        if (response.size() > 0)
            return response.at(0).image_data_uint8;
        else
            return std::vector<uint8_t>();
    }

    virtual void setCarControls(const CarApiBase::CarControls& controls) override
    {
        UWheeledVehicleMovementComponent* movement = car_pawn_->GetVehicleMovementComponent();
        movement->SetThrottleInput(controls.throttle);
        movement->SetSteeringInput(controls.steering);
        movement->SetBrakeInput(controls.brake);
        movement->SetHandbrakeInput(controls.handbrake);

            movement->SetUseAutoGears(!controls.is_manual_gear);
        if (!controls.is_manual_gear && movement->GetTargetGear() < 0)
            movement->SetTargetGear(0, true); //in auto gear we must have gear >= 0
        if (controls.is_manual_gear && movement->GetTargetGear() != controls.manual_gear)
            movement->SetTargetGear(controls.manual_gear, controls.gear_immediate);
    }

    virtual CarApiBase::CarState getCarState() override
    {
        CarApiBase::CarState state(
            car_pawn_->GetVehicleMovement()->GetForwardSpeed(),
            car_pawn_->GetVehicleMovement()->GetCurrentGear(),
            NedTransform::toNedMeters(car_pawn_->GetActorLocation(), true),
            NedTransform::toNedMeters(car_pawn_->GetVelocity(), true),
            NedTransform::toQuaternionr(car_pawn_->GetActorRotation().Quaternion(), true));
        return state;
    }

    virtual void reset() override
    {
        UAirBlueprintLib::RunCommandOnGameThread([this]() {
            this->car_pawn_->reset(false);
        });
    }

    virtual void simSetPose(const Pose& pose, bool ignore_collison) override
    {
        UAirBlueprintLib::RunCommandOnGameThread([this, pose, ignore_collison]() {
            this->car_pawn_->getVehiclePawnWrapper()->setPose(pose, ignore_collison);
        });
    }

    virtual Pose simGetPose() override
    {
        return this->car_pawn_->getVehiclePawnWrapper()->getPose();
    }

    virtual msr::airlib::GeoPoint getHomeGeoPoint() override
    {
        return car_pawn_->getVehiclePawnWrapper()->getHomePoint();
    }

    virtual void enableApiControl(bool is_enabled) override
    {
        car_pawn_->enableApiControl(is_enabled);
    }

    virtual bool isApiControlEnabled() override
    {
        return car_pawn_->isApiControlEnabled();
    }

    virtual ~CarController() = default;

private:
    ACarPawn* car_pawn_;
};

ACarPawn::ACarPawn()
{
    this->AutoPossessPlayer = EAutoReceiveInput::Player0;
    //this->AutoReceiveInput = EAutoReceiveInput::Player0;

    // Car mesh
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(TEXT("/AirSim/VehicleAdv/Vehicle/Vehicle_SkelMesh.Vehicle_SkelMesh"));
    GetMesh()->SetSkeletalMesh(CarMesh.Object);
    
    static ConstructorHelpers::FClassFinder<UObject> AnimBPClass(TEXT("/AirSim/VehicleAdv/Vehicle/VehicleAnimationBlueprint"));
    GetMesh()->SetAnimationMode(EAnimationMode::AnimationBlueprint);
    GetMesh()->SetAnimInstanceClass(AnimBPClass.Class);

    // Setup friction materials
    static ConstructorHelpers::FObjectFinder<UPhysicalMaterial> SlipperyMat(TEXT("/AirSim/VehicleAdv/PhysicsMaterials/Slippery.Slippery"));
    SlipperyMaterial = SlipperyMat.Object;
        
    static ConstructorHelpers::FObjectFinder<UPhysicalMaterial> NonSlipperyMat(TEXT("/AirSim/VehicleAdv/PhysicsMaterials/NonSlippery.NonSlippery"));
    NonSlipperyMaterial = NonSlipperyMat.Object;

    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

    UWheeledVehicleMovementComponent4W* Vehicle4W = CastChecked<UWheeledVehicleMovementComponent4W>(GetVehicleMovement());

    check(Vehicle4W->WheelSetups.Num() == 4);

    // Wheels/Tyres
    // Setup the wheels
    Vehicle4W->WheelSetups[0].WheelClass = UCarWheelFront::StaticClass();
    Vehicle4W->WheelSetups[0].BoneName = FName("PhysWheel_FL");
    Vehicle4W->WheelSetups[0].AdditionalOffset = FVector(0.f, -8.f, 0.f);

    Vehicle4W->WheelSetups[1].WheelClass = UCarWheelFront::StaticClass();
    Vehicle4W->WheelSetups[1].BoneName = FName("PhysWheel_FR");
    Vehicle4W->WheelSetups[1].AdditionalOffset = FVector(0.f, 8.f, 0.f);

    Vehicle4W->WheelSetups[2].WheelClass = UCarWheelRear::StaticClass();
    Vehicle4W->WheelSetups[2].BoneName = FName("PhysWheel_BL");
    Vehicle4W->WheelSetups[2].AdditionalOffset = FVector(0.f, -8.f, 0.f);

    Vehicle4W->WheelSetups[3].WheelClass = UCarWheelRear::StaticClass();
    Vehicle4W->WheelSetups[3].BoneName = FName("PhysWheel_BR");
    Vehicle4W->WheelSetups[3].AdditionalOffset = FVector(0.f, 8.f, 0.f);

    // Adjust the tire loading
    Vehicle4W->MinNormalizedTireLoad = 0.0f;
    Vehicle4W->MinNormalizedTireLoadFiltered = 0.2f;
    Vehicle4W->MaxNormalizedTireLoad = 2.0f;
    Vehicle4W->MaxNormalizedTireLoadFiltered = 2.0f;

    // Engine 
    // Torque setup
    Vehicle4W->MaxEngineRPM = 5700.0f;
    Vehicle4W->EngineSetup.TorqueCurve.GetRichCurve()->Reset();
    Vehicle4W->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(0.0f, 400.0f);
    Vehicle4W->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(1890.0f, 500.0f);
    Vehicle4W->EngineSetup.TorqueCurve.GetRichCurve()->AddKey(5730.0f, 400.0f);
 
    // Adjust the steering 
    Vehicle4W->SteeringCurve.GetRichCurve()->Reset();
    Vehicle4W->SteeringCurve.GetRichCurve()->AddKey(0.0f, 1.0f);
    Vehicle4W->SteeringCurve.GetRichCurve()->AddKey(40.0f, 0.7f);
    Vehicle4W->SteeringCurve.GetRichCurve()->AddKey(120.0f, 0.6f);
            
    // Transmission	
    // We want 4wd
    Vehicle4W->DifferentialSetup.DifferentialType = EVehicleDifferential4W::LimitedSlip_4W;
    
    // Drive the front wheels a little more than the rear
    Vehicle4W->DifferentialSetup.FrontRearSplit = 0.65;

    // Automatic gearbox
    Vehicle4W->TransmissionSetup.bUseGearAutoBox = true;
    Vehicle4W->TransmissionSetup.GearSwitchTime = 0.15f;
    Vehicle4W->TransmissionSetup.GearAutoBoxLatency = 1.0f;

    // Disable reverse as brake, this is needed for SetBreakInput() to take effect
    Vehicle4W->bReverseAsBrake = false;

    // Physics settings
    // Adjust the center of mass - the buggy is quite low
    UPrimitiveComponent* UpdatedPrimitive = Cast<UPrimitiveComponent>(Vehicle4W->UpdatedComponent);
    if (UpdatedPrimitive)
    {
        UpdatedPrimitive->BodyInstance.COMNudge = FVector(8.0f, 0.0f, 0.0f);
    }

    // Set the inertia scale. This controls how the mass of the vehicle is distributed.
    Vehicle4W->InertiaTensorScale = FVector(1.0f, 1.333f, 1.2f);

    // Create In-Car camera component 
    InternalCameraBase1 = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase1"));
    InternalCameraBase1->SetRelativeLocation(FVector(36.0f, 0, 50.0f)); //center
    InternalCameraBase1->SetupAttachment(GetMesh());
    InternalCameraBase2 = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase2"));
    InternalCameraBase2->SetRelativeLocation(FVector(36.0f, -10, 50.0f)); //left
    InternalCameraBase2->SetupAttachment(GetMesh());
    InternalCameraBase3 = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase3"));
    InternalCameraBase3->SetRelativeLocation(FVector(36.0f, 10, 50.0f)); //right
    InternalCameraBase3->SetupAttachment(GetMesh());
    InternalCameraBase4 = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase4"));
    InternalCameraBase4->SetRelativeLocation(FVector(25, -10, 75.0f)); //driver
    InternalCameraBase4->SetupAttachment(GetMesh());
    InternalCameraBase5 = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase5"));
    InternalCameraBase5->SetRelativeLocation(FVector(-36.0f, 0, 50.0f)); //rear
    InternalCameraBase5->SetRelativeRotation(FRotator(0, 180, 0));
    InternalCameraBase5->SetupAttachment(GetMesh());

    // In car HUD
    // Create text render component for in car speed display
    InCarSpeed = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarSpeed"));
    InCarSpeed->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    InCarSpeed->SetRelativeLocation(FVector(35.0f, -6.0f, 20.0f));
    InCarSpeed->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    InCarSpeed->SetupAttachment(GetMesh());
    InCarSpeed->SetVisibility(true);

    // Create text render component for in car gear display
    InCarGear = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
    InCarGear->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
    InCarGear->SetRelativeLocation(FVector(35.0f, 5.0f, 20.0f));
    InCarGear->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
    InCarGear->SetupAttachment(GetMesh());
    InCarGear->SetVisibility(true);

    // Setup the audio component and allocate it a sound cue
    static ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/AirSim/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
    EngineSoundComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
    EngineSoundComponent->SetSound(SoundCue.Object);
    EngineSoundComponent->SetupAttachment(GetMesh());

    // Colors for the in-car gear display. One for normal one for reverse
    GearDisplayReverseColor = FColor(255, 0, 0, 255);
    GearDisplayColor = FColor(255, 255, 255, 255);

    bIsLowFriction = false;
    bInReverseGear = false;

    wrapper_.reset(new VehiclePawnWrapper());
}

void ACarPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    wrapper_->onCollision(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void ACarPawn::initializeForBeginPlay(bool enable_rpc, const std::string& api_server_address, bool engine_sound)
{
    if (engine_sound)
        EngineSoundComponent->Activate(); 
    else
        EngineSoundComponent->Deactivate(); 

    //put camera little bit above vehicle
    FTransform camera_transform(FVector::ZeroVector);
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
    InternalCamera1 = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    InternalCamera1->AttachToComponent(InternalCameraBase1, FAttachmentTransformRules::KeepRelativeTransform);
    InternalCamera2 = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    InternalCamera2->AttachToComponent(InternalCameraBase2, FAttachmentTransformRules::KeepRelativeTransform);
    InternalCamera3 = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    InternalCamera3->AttachToComponent(InternalCameraBase3, FAttachmentTransformRules::KeepRelativeTransform);
    InternalCamera4 = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
    InternalCamera4->AttachToComponent(InternalCameraBase4, FAttachmentTransformRules::KeepRelativeTransform);
    InternalCamera5 = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, FTransform(FRotator(0, 180, 0), FVector::ZeroVector), camera_spawn_params);
    InternalCamera5->AttachToComponent(InternalCameraBase4, FAttachmentTransformRules::KeepRelativeTransform);


    setupInputBindings();

    std::vector<APIPCamera*> cameras = { InternalCamera1, InternalCamera2, InternalCamera3, InternalCamera4, InternalCamera5 };
    wrapper_->initialize(this, cameras);

    startApiServer(enable_rpc, api_server_address);
}

void ACarPawn::reset(bool disable_api_control)
{
    this->getVehiclePawnWrapper()->reset();
    controller_->setCarControls(CarController::CarControls());

    if (disable_api_control)
        api_control_enabled_ = false;
}

void ACarPawn::enableApiControl(bool is_enabled)
{
    api_control_enabled_ = is_enabled;
}

bool ACarPawn::isApiControlEnabled()
{
    return api_control_enabled_;
}

void ACarPawn::startApiServer(bool enable_rpc, const std::string& api_server_address)
{
    if (enable_rpc) {
        controller_.reset(new CarController(this));


#ifdef AIRLIB_NO_RPC
        rpclib_server_.reset(new msr::airlib::DebugApiServer());
#else
        rpclib_server_.reset(new msr::airlib::CarRpcLibServer(controller_.get(), api_server_address));
#endif

        rpclib_server_->start();
        UAirBlueprintLib::LogMessageString("API server started at ",
            api_server_address == "" ? "(default)" : api_server_address.c_str(), LogDebugLevel::Informational);
    }
    else
        UAirBlueprintLib::LogMessageString("API server is disabled in settings", "", LogDebugLevel::Informational);

}
void ACarPawn::stopApiServer()
{
    if (rpclib_server_ != nullptr) {
        rpclib_server_->stop();
        rpclib_server_.reset(nullptr);
        controller_.reset(nullptr);
    }
}

bool ACarPawn::isApiServerStarted()
{
    return rpclib_server_ != nullptr;
}

void ACarPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    stopApiServer();

    if (InternalCamera1)
        InternalCamera1->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
    InternalCamera1 = nullptr;
    if (InternalCamera2)
        InternalCamera2->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
    InternalCamera2 = nullptr;
    if (InternalCamera3)
        InternalCamera3->DetachFromActor(FDetachmentTransformRules::KeepRelativeTransform);
    InternalCamera3 = nullptr;
}


VehiclePawnWrapper* ACarPawn::getVehiclePawnWrapper()
{
    return wrapper_.get();
}

void ACarPawn::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Up, 1), this,
        this, &ACarPawn::MoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Down, -1), this,
        this, &ACarPawn::MoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Right, 1), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Left, -1), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::OnHandbrakePressed, true);
    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::OnHandbrakeReleased, false);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::SpaceBar, 1), this,
        this, &ACarPawn::FootBrake);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Gamepad_LeftX, 1), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Gamepad_RightY, -1), this,
        this, &ACarPawn::MoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::Gamepad_RightTriggerAxis, 1), this,
        this, &ACarPawn::FootBrake);

    UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::OnReversePressed, true);
    UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::OnReverseReleased, false);
}

void ACarPawn::MoveForward(float Val)
{
    //if (Val < 0)
    //    OnReversePressed();
    //else
    //    OnReverseReleased();

    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Throttle: "), FString::SanitizeFloat(Val), LogDebugLevel::Informational);

        GetVehicleMovementComponent()->SetThrottleInput(Val);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Throttle: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::MoveRight(float Val)
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Steering: "), FString::SanitizeFloat(Val), LogDebugLevel::Informational);

        GetVehicleMovementComponent()->SetSteeringInput(Val);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Steering: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::OnHandbrakePressed()
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Handbrake: "), TEXT("Pressed"), LogDebugLevel::Informational);

        GetVehicleMovementComponent()->SetHandbrakeInput(true);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Handbrake: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::OnHandbrakeReleased()
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Handbrake: "), TEXT("Released"), LogDebugLevel::Informational);

        GetVehicleMovementComponent()->SetHandbrakeInput(false);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Handbrake: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::FootBrake(float Val)
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Footbrake: "), FString::SanitizeFloat(Val), LogDebugLevel::Informational);

        GetVehicleMovementComponent()->SetBrakeInput(Val);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Footbrake: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::OnReversePressed()
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Reverse: "), TEXT("Pressed"), LogDebugLevel::Informational);

        if (GetVehicleMovementComponent()->GetTargetGear() >= 0)
            GetVehicleMovementComponent()->SetTargetGear(-1, true);
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Reverse: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::OnReverseReleased()
{
    if (!api_control_enabled_) {
        UAirBlueprintLib::LogMessage(TEXT("Reverse: "), TEXT("Released"), LogDebugLevel::Informational);

        if (GetVehicleMovementComponent()->GetTargetGear() < 0) {
            GetVehicleMovementComponent()->SetTargetGear(0, true);
            GetVehicleMovementComponent()->SetUseAutoGears(true);
        }
    }
    else
        UAirBlueprintLib::LogMessage(TEXT("Reverse: "), TEXT("(API)"), LogDebugLevel::Informational);
}

void ACarPawn::Tick(float Delta)
{
    Super::Tick(Delta);

    // Setup the flag to say we are in reverse gear
    bInReverseGear = GetVehicleMovement()->GetCurrentGear() < 0;
    
    // Update phsyics material
    UpdatePhysicsMaterial();

    // Update the strings used in the hud (incar and onscreen)
    UpdateHUDStrings();

    // Set the string in the incar hud
    UpdateInCarHUD();

    // Pass the engine RPM to the sound component
    float RPMToAudioScale = 2500.0f / GetVehicleMovement()->GetEngineMaxRotationSpeed();
    EngineSoundComponent->SetFloatParameter(EngineAudioRPM, GetVehicleMovement()->GetEngineRotationSpeed()*RPMToAudioScale);
}

void ACarPawn::BeginPlay()
{
    Super::BeginPlay();

    // Start an engine sound playing
    EngineSoundComponent->Play();
}

void ACarPawn::UpdateHUDStrings()
{
    float KPH = FMath::Abs(GetVehicleMovement()->GetForwardSpeed()) * 0.036f;
    int32 KPH_int = FMath::FloorToInt(KPH);
    int32 Gear = GetVehicleMovement()->GetCurrentGear();

    // Using FText because this is display text that should be localizable
    SpeedDisplayString = FText::Format(LOCTEXT("SpeedFormat", "{0} km/h"), FText::AsNumber(KPH_int));


    if (bInReverseGear == true)
    {
        GearDisplayString = FText(LOCTEXT("ReverseGear", "R"));
    }
    else
    {
        GearDisplayString = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
    }


    UAirBlueprintLib::LogMessage(TEXT("Speed: "), SpeedDisplayString.ToString(), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessage(TEXT("Gear: "), GearDisplayString.ToString(), LogDebugLevel::Informational);

}

void ACarPawn::UpdateInCarHUD()
{
    APlayerController* PlayerController = Cast<APlayerController>(GetController());
    if ((PlayerController != nullptr) && (InCarSpeed != nullptr) && (InCarGear != nullptr))
    {
        // Setup the text render component strings
        InCarSpeed->SetText(SpeedDisplayString);
        InCarGear->SetText(GearDisplayString);
        
        if (bInReverseGear == false)
        {
            InCarGear->SetTextRenderColor(GearDisplayColor);
        }
        else
        {
            InCarGear->SetTextRenderColor(GearDisplayReverseColor);
        }
    }
}

void ACarPawn::UpdatePhysicsMaterial()
{
    if (GetActorUpVector().Z < 0)
    {
        if (bIsLowFriction == true)
        {
            GetMesh()->SetPhysMaterialOverride(NonSlipperyMaterial);
            bIsLowFriction = false;
        }
        else
        {
            GetMesh()->SetPhysMaterialOverride(SlipperyMaterial);
            bIsLowFriction = true;
        }
    }
}

#undef LOCTEXT_NAMESPACE
