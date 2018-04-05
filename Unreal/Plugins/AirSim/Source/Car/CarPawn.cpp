#include "CarPawn.h"
#include "CarWheelFront.h"
#include "CarWheelRear.h"
#include "common/common_utils/Utils.hpp"
#include "Components/TextRenderComponent.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Controller.h"
#include "AirBlueprintLib.h"
#include "common/ClockFactory.hpp"
#include "PIPCamera.h"
#include <vector>



// Needed for VR Headset
#if HMD_MODULE_INCLUDED
#include "IHeadMountedDisplay.h"
#endif // HMD_MODULE_INCLUDED

const FName ACarPawn::LookUpBinding("LookUp");
const FName ACarPawn::LookRightBinding("LookRight");
const FName ACarPawn::EngineAudioRPM("RPM");

#define LOCTEXT_NAMESPACE "VehiclePawn"


ACarPawn::ACarPawn()
{
    this->AutoPossessPlayer = EAutoReceiveInput::Player0;
    //this->AutoReceiveInput = EAutoReceiveInput::Player0;

    UWheeledVehicleMovementComponent4W* Vehicle4W = CastChecked<UWheeledVehicleMovementComponent4W>(GetVehicleMovement());

    // load assets
    if (UseDefaultMesh) {
        static MeshContructionHelpers helpers(AirSimSettings::singleton().car_mesh_paths);
        GetMesh()->SetSkeletalMesh(helpers.skeleton);
        GetMesh()->SetAnimationMode(EAnimationMode::AnimationBlueprint);
        GetMesh()->SetAnimInstanceClass(helpers.bp->GeneratedClass);
        SlipperyMaterial = helpers.slippery_mat;
        NonSlipperyMaterial = helpers.non_slippery_mat;
    }

    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;

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
    Vehicle4W->MinNormalizedTireLoadFiltered = 0.2308f;
    Vehicle4W->MaxNormalizedTireLoad = 2.0f;
    Vehicle4W->MaxNormalizedTireLoadFiltered = 2.0f;

    // Engine 
    // Torque setup
    Vehicle4W->EngineSetup.MaxRPM = 5700.0f;
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
    Vehicle4W->bDeprecatedSpringOffsetMode = true;

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
    ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/AirSim/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
    EngineSoundComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
    EngineSoundComponent->SetSound(SoundCue.Object);
    EngineSoundComponent->SetupAttachment(GetMesh());

    // Colors for the in-car gear display. One for normal one for reverse
    GearDisplayReverseColor = FColor(255, 0, 0, 255);
    GearDisplayColor = FColor(255, 255, 255, 255);

    bIsLowFriction = false;

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
    InternalCamera5->AttachToComponent(InternalCameraBase5, FAttachmentTransformRules::KeepRelativeTransform);


    setupInputBindings();

    std::vector<APIPCamera*> cameras = { InternalCamera1, InternalCamera2, InternalCamera3, InternalCamera4, InternalCamera5 };
    wrapper_->initialize(this, cameras);
    wrapper_->setKinematics(&kinematics_);

    startApiServer(enable_rpc, api_server_address);

    //TODO: should do reset() here?
    keyboard_controls_ = joystick_controls_ = CarPawnApi::CarControls();

    //joystick
    if (wrapper_->getRemoteControlID() >= 0) {
        joystick_.getJoyStickState(wrapper_->getRemoteControlID(), joystick_state_);
        if (joystick_state_.is_initialized)
            UAirBlueprintLib::LogMessageString("RC Controller on USB: ", joystick_state_.pid_vid == "" ?
                "(Detected)" : joystick_state_.pid_vid, LogDebugLevel::Informational);
        else
            UAirBlueprintLib::LogMessageString("RC Controller on USB not detected: ", 
                std::to_string(joystick_state_.connection_error_code), LogDebugLevel::Informational);
    }

}

void ACarPawn::reset(bool disable_api_control)
{
    keyboard_controls_ = joystick_controls_ = CarPawnApi::CarControls();
    api_->reset();

    if (disable_api_control)
        api_->enableApiControl(false);
}

void ACarPawn::startApiServer(bool enable_rpc, const std::string& api_server_address)
{
    if (enable_rpc) {
        api_.reset(new CarPawnApi(getVehiclePawnWrapper(), this->GetVehicleMovement()));


#ifdef AIRLIB_NO_RPC
        rpclib_server_.reset(new msr::airlib::DebugApiServer());
#else
        rpclib_server_.reset(new msr::airlib::CarRpcLibServer(api_.get(), api_server_address));
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
        api_.reset(nullptr);
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

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Right, 0.5), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Left, -0.5), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::OnHandbrakePressed, true);
    UAirBlueprintLib::BindActionToKey("Handbrake", EKeys::End, this, &ACarPawn::OnHandbrakeReleased, false);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::SpaceBar, 1), this,
        this, &ACarPawn::FootBrake);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveRight", EKeys::Gamepad_LeftX, 1), this,
        this, &ACarPawn::MoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("MoveForward", EKeys::Gamepad_RightTriggerAxis, 1), this,
        this, &ACarPawn::MoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("Footbrake", EKeys::Gamepad_LeftTriggerAxis, 1), this,
        this, &ACarPawn::FootBrake);

    //below is not needed
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::OnReversePressed, true);
    //UAirBlueprintLib::BindActionToKey("Reverse", EKeys::Down, this, &ACarPawn::OnReverseReleased, false);
}

void ACarPawn::MoveForward(float Val)
{
    if (Val < 0)
        OnReversePressed();
    else
        OnReverseReleased();

    keyboard_controls_.throttle = Val;
}

void ACarPawn::MoveRight(float Val)
{
    keyboard_controls_.steering = Val;
}

void ACarPawn::OnHandbrakePressed()
{
    keyboard_controls_.handbrake = true;
}

void ACarPawn::OnHandbrakeReleased()
{
    keyboard_controls_.handbrake = false;
}

void ACarPawn::FootBrake(float Val)
{
    keyboard_controls_.brake = Val;
}

void ACarPawn::OnReversePressed()
{
    if (keyboard_controls_.manual_gear >= 0) {
        keyboard_controls_.is_manual_gear = true;
        keyboard_controls_.manual_gear = -1;
        keyboard_controls_.gear_immediate = true;
    }
}

void ACarPawn::OnReverseReleased()
{
    if (keyboard_controls_.manual_gear < 0) {
        keyboard_controls_.is_manual_gear = false;
        keyboard_controls_.manual_gear = 0;
        keyboard_controls_.gear_immediate = true;
    }
}

void ACarPawn::updateKinematics(float delta)
{
    auto last_kinematics = kinematics_;

    kinematics_.pose = getVehiclePawnWrapper()->getPose();
    kinematics_.twist.linear = wrapper_->getNedTransform().toNedMeters(this->GetVelocity(), false);
    kinematics_.twist.angular = msr::airlib::VectorMath::toAngularVelocity(
        kinematics_.pose.orientation, last_kinematics.pose.orientation, delta);

    kinematics_.accelerations.linear = (kinematics_.twist.linear - last_kinematics.twist.linear) / delta;
    kinematics_.accelerations.angular = (kinematics_.twist.angular - last_kinematics.twist.angular) / delta;

    //TODO: update other fields

}

void ACarPawn::Tick(float Delta)
{
    Super::Tick(Delta);

    updateCarControls();

    updateKinematics(Delta);

    // Update phsyics material
    UpdatePhysicsMaterial();

    // Update the strings used in the hud (incar and onscreen)
    UpdateHUDStrings();

    // Set the string in the incar hud
    UpdateInCarHUD();

    // Pass the engine RPM to the sound component
    float RPMToAudioScale = 2500.0f / GetVehicleMovement()->GetEngineMaxRotationSpeed();
    EngineSoundComponent->SetFloatParameter(EngineAudioRPM, GetVehicleMovement()->GetEngineRotationSpeed()*RPMToAudioScale);

    getVehiclePawnWrapper()->setLogLine(getLogString());
}

void ACarPawn::updateCarControls()
{
    if (wrapper_->getRemoteControlID() >= 0 && joystick_state_.is_initialized) {
        joystick_.getJoyStickState(0, joystick_state_);

        if ((joystick_state_.buttons & 4) | (joystick_state_.buttons & 1024)) { //X button or Start button
            reset();
            return;
        }

        joystick_controls_.steering = joystick_state_.left_y * 1.25;
        joystick_controls_.throttle = (-joystick_state_.right_x + 1) / 2;
        joystick_controls_.brake = -joystick_state_.right_z + 1;

        //Two steel levers behind wheel
        joystick_controls_.handbrake = (joystick_state_.buttons & 32) | (joystick_state_.buttons & 64) ? 1 : 0;

        if ((joystick_state_.buttons & 256) | (joystick_state_.buttons & 2)) { //RSB button or B button
            joystick_controls_.manual_gear = -1;
            joystick_controls_.is_manual_gear = true;
            joystick_controls_.gear_immediate = true;
        }
        else if ((joystick_state_.buttons & 512) | (joystick_state_.buttons & 1)) { //LSB button or A button
            joystick_controls_.manual_gear = 0;
            joystick_controls_.is_manual_gear = false;
            joystick_controls_.gear_immediate = true;
        }

        UAirBlueprintLib::LogMessageString("Control Mode: ", "Wheel/Joystick", LogDebugLevel::Informational);
        current_controls_ = joystick_controls_;
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "Keyboard", LogDebugLevel::Informational);
        current_controls_ = keyboard_controls_;
    }

    if (!api_->isApiControlEnabled()) {
        api_->setCarControls(current_controls_);
    }
    else {
        UAirBlueprintLib::LogMessageString("Control Mode: ", "API", LogDebugLevel::Informational);
        current_controls_ = api_->getCarControls();
    }
    UAirBlueprintLib::LogMessageString("Accel: ", std::to_string(current_controls_.throttle), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Break: ", std::to_string(current_controls_.brake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Steering: ", std::to_string(current_controls_.steering), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Handbreak: ", std::to_string(current_controls_.handbrake), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessageString("Target Gear: ", std::to_string(current_controls_.manual_gear), LogDebugLevel::Informational);
}

void ACarPawn::BeginPlay()
{
    Super::BeginPlay();

    // Start an engine sound playing
    EngineSoundComponent->Play();
}

void ACarPawn::UpdateHUDStrings()
{
    float vel = FMath::Abs(GetVehicleMovement()->GetForwardSpeed() / 100); //cm/s -> m/s
    float vel_rounded = FMath::FloorToInt(vel * 10) / 10.0f;
    int32 Gear = GetVehicleMovement()->GetCurrentGear();

    // Using FText because this is display text that should be localizable
    SpeedDisplayString = FText::Format(LOCTEXT("SpeedFormat", "{0} m/s"), FText::AsNumber(vel_rounded));


    if (GetVehicleMovement()->GetCurrentGear() < 0)
    {
        GearDisplayString = FText(LOCTEXT("ReverseGear", "R"));
    }
    else
    {
        GearDisplayString = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
    }


    UAirBlueprintLib::LogMessage(TEXT("Speed: "), SpeedDisplayString.ToString(), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessage(TEXT("Gear: "), GearDisplayString.ToString(), LogDebugLevel::Informational);
    UAirBlueprintLib::LogMessage(TEXT("RPM: "), FText::AsNumber(GetVehicleMovement()->GetEngineRotationSpeed()).ToString(), LogDebugLevel::Informational);


}

void ACarPawn::UpdateInCarHUD()
{
    APlayerController* PlayerController = Cast<APlayerController>(GetController());
    if ((PlayerController != nullptr) && (InCarSpeed != nullptr) && (InCarGear != nullptr))
    {
        // Setup the text render component strings
        InCarSpeed->SetText(SpeedDisplayString);
        InCarGear->SetText(GearDisplayString);

        if (GetVehicleMovement()->GetCurrentGear() >= 0)
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

std::string ACarPawn::getLogString()
{
    // Timestamp \t Speed \t Throttle \t Steering \t Brake \t gear

    // timestamp
    uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);

    // Speed
    float KPH = FMath::Abs(GetVehicleMovement()->GetForwardSpeed()) * 0.036f;
    int32 KPH_int = FMath::FloorToInt(KPH);

    // Gear
    FString gearString = GearDisplayString.ToString();
    std::string gear = std::string(TCHAR_TO_UTF8(*gearString));

    std::string logString = std::to_string(timestamp_millis).append("\t")
        .append(std::to_string(KPH_int).append("\t"))
        .append(std::to_string(current_controls_.throttle)).append("\t")
        .append(std::to_string(current_controls_.steering)).append("\t")
        .append(std::to_string(current_controls_.brake)).append("\t")
        .append(gear).append("\t");

    return logString;
}

#undef LOCTEXT_NAMESPACE
