#include "FlyingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "VehicleSimApi.h"

AFlyingPawn::AFlyingPawn()
{
    static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class(TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Class : nullptr;
}

void AFlyingPawn::BeginPlay()
{
    Super::BeginPlay();
}
void AFlyingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    vehicle_sim_api_.reset();
    cameras_.clear();
    Super::EndPlay(EndPlayReason);
}

void AFlyingPawn::initializeForBeginPlay(const NedTransform& global_transform, const std::string& vehicle_name)
{
    cameras_.clear();
    vehicle_sim_api_.reset(new VehicleSimApi(this, &cameras_, global_transform, vehicle_name));

    //get references of existing camera
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

    cameras_["0"] = cameras_["front_center"] = fpv_camera_front_center_;
    cameras_["1"] = cameras_["front_right"] = fpv_camera_front_right_;
    cameras_["2"] = cameras_["front_left"] = fpv_camera_front_left_;
    cameras_["3"] = cameras_["bottom_center"] = fpv_camera_bottom_center_;
    cameras_["4"] = cameras_["back_center"] = fpv_camera_back_center_;
    
    //UStaticMeshComponent* bodyMesh = UAirBlueprintLib::GetActorComponent<UStaticMeshComponent>(this, TEXT("BodyMesh"));
    USceneComponent* bodyMesh = GetRootComponent();
    FActorSpawnParameters camera_spawn_params;
    camera_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    const auto& transform = vehicle_sim_api_->getNedTransform();
    for (const auto& camera_setting_pair : vehicle_sim_api_->getVehicleSettings().cameras) {
        const auto& setting = camera_setting_pair.second;

        FVector position = transform.fromLocalNed(
            NedTransform::Vector3r(setting.position.x, setting.position.y, setting.position.z))
            - transform.fromLocalNed(NedTransform::Vector3r(0.0, 0.0, 0.0));

        FTransform camera_transform(FRotator(setting.rotation.pitch, setting.rotation.yaw, setting.rotation.roll),
            position, FVector(1., 1., 1.));

        APIPCamera* camera = this->GetWorld()->SpawnActor<APIPCamera>(pip_camera_class_, camera_transform, camera_spawn_params);
        camera->AttachToComponent(bodyMesh, FAttachmentTransformRules::KeepRelativeTransform);

        cameras_[camera_setting_pair.first] = camera;
    }

    for (auto i = 0; i < rotor_count; ++i) {
        rotating_movements_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rotation") + FString::FromInt(i));
    }

    vehicle_sim_api_->setupCamerasFromSettings();
}

void AFlyingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation, 
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    vehicle_sim_api_->onCollision(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
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

