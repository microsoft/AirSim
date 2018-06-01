#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"


AFlyingPawn::AFlyingPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;
}

void AFlyingPawn::initializeForBeginPlay(msr::airlib::VehicleSimApiBase* sim_api, const AdditionalCameraSettings& additionalCameras)
{
    sim_api_ = sim_api;

    //get references of components so we can use later
    setupComponentReferences(additionalCameras);

    std::vector<APIPCamera*> cameras = {fpv_camera_front_center_, fpv_camera_front_right_, fpv_camera_front_left_, 
        fpv_camera_bottom_center_, fpv_camera_back_center_};
    for (APIPCamera* camera : AdditionalCameras) {
        cameras.push_back(camera);
    }
}

void AFlyingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    sim_api_->onCollision(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

void AFlyingPawn::setRotorSpeed(int rotor_index, float radsPerSec)
{
    if (rotor_index >= 0 && rotor_index < rotor_count) {
        auto comp = rotating_movements_[rotor_index];
        if (comp != nullptr) {
            comp->RotationRate.Yaw = radsPerSec * 180.0f / M_PIf * RotatorFactor;
        }
    }
}

void AFlyingPawn::setupComponentReferences(const std::vector<msr::airlib::AirSimSettings::AdditionalCameraSetting>& additionalCameras)
{
    fpv_camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    fpv_camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    fpv_camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    fpv_camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    fpv_camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());

    //UStaticMeshComponent* bodyMesh = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("BodyMesh"));
    USceneComponent* bodyMesh = GetRootComponent();
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    // Can't obtain NedTransform from sim_api because it's not initialized yet, so make our own.
    NedTransform transform;
    transform.initialize(this);

    for (const msr::airlib::AirSimSettings::AdditionalCameraSetting& setting : additionalCameras) {
        FVector position = transform.toNeuUU(NedTransform::Vector3r(setting.x, setting.y, setting.z)) - transform.toNeuUU(NedTransform::Vector3r(0.0, 0.0, 0.0));
        FTransform camera_transform(FRotator(setting.pitch, setting.yaw, setting.roll), position, FVector(1., 1., 1.));
        APIPCamera* camera = GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
        camera->AttachToComponent(bodyMesh, FAttachmentTransformRules::KeepRelativeTransform);
        AdditionalCameras.Add(camera);
    }

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }
}
