// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirBlueprintLib.h"
#include "GameFramework/WorldSettings.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkinnedMeshComponent.h"
#include "GameFramework/RotatingMovementComponent.h"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "UObject/UObjectIterator.h"
#include "Camera/CameraComponent.h"
//#include "Runtime/Foliage/Public/FoliageType.h"
#include "Misc/MessageDialog.h"
#include "Engine/LocalPlayer.h"
#include "Engine/SkeletalMesh.h"
#include "Slate/SceneViewport.h"
#include "IImageWrapper.h"
#include "Misc/ObjectThumbnail.h"
#include "Engine/Engine.h"
#include <exception>
#include "common/common_utils/Utils.hpp"

/*
//TODO: change naming conventions to same as other files?
Naming conventions in this file:
Methods -> CamelCase
parameters -> camel_case
*/

bool UAirBlueprintLib::log_messages_hidden_ = false;
msr::airlib::AirSimSettings::SegmentationSetting::MeshNamingMethodType UAirBlueprintLib::mesh_naming_method_ =
    msr::airlib::AirSimSettings::SegmentationSetting::MeshNamingMethodType::OwnerName;
IImageWrapperModule* UAirBlueprintLib::image_wrapper_module_ = nullptr;

void UAirBlueprintLib::LogMessageString(const std::string &prefix, const std::string &suffix, LogDebugLevel level, float persist_sec)
{
    LogMessage(FString(prefix.c_str()), FString(suffix.c_str()), level, persist_sec);
}

EAppReturnType::Type UAirBlueprintLib::ShowMessage(EAppMsgType::Type message_type, const std::string& message, const std::string& title)
{
    FText title_text = FText::FromString(title.c_str());

    return FMessageDialog::Open(message_type,
        FText::FromString(message.c_str()),
        &title_text);
}

void UAirBlueprintLib::enableWorldRendering(AActor* context, bool enable)
{
    ULocalPlayer* player = context->GetWorld()->GetFirstLocalPlayerFromController();
    if (player)
    {
        UGameViewportClient* viewport_client = player->ViewportClient;
        if (viewport_client)
        {
            viewport_client->bDisableWorldRendering = enable;
        }
    }
}

void UAirBlueprintLib::setSimulatePhysics(AActor* actor, bool simulate_physics)
{
    TInlineComponentArray<UPrimitiveComponent*> components;
    actor->GetComponents(components);

    for (UPrimitiveComponent* component : components)
    {
        component->SetSimulatePhysics(simulate_physics);
    }
}

std::vector<UPrimitiveComponent*> UAirBlueprintLib::getPhysicsComponents(AActor* actor)
{
    std::vector<UPrimitiveComponent*> phys_comps;
    TInlineComponentArray<UPrimitiveComponent*> components;
    actor->GetComponents(components);

    for (UPrimitiveComponent* component : components)
    {
        if (component->IsSimulatingPhysics())
            phys_comps.push_back(component);
    }

    return phys_comps;
}

void UAirBlueprintLib::resetSimulatePhysics(AActor* actor)
{
    TInlineComponentArray<UPrimitiveComponent*> components;
    actor->GetComponents(components);

    for (UPrimitiveComponent* component : components)
    {
        if (component->IsSimulatingPhysics()) {
            component->SetSimulatePhysics(false);
            component->SetSimulatePhysics(true);
        }
    }
}

void UAirBlueprintLib::enableViewportRendering(AActor* context, bool enable)
{
    // Enable/disable primary viewport rendering flag
    auto* viewport = context->GetWorld()->GetGameViewport();
    if (!viewport)
        return;

    if (!enable) {
        // This disables rendering of the main viewport in the same way as the
        // console command "show rendering" would do.
        viewport->EngineShowFlags.SetRendering(false);

        // When getting an image through the API, the image is produced after the render
        // thread has finished rendering the current and the subsequent frame. This means
        // that the frame rate for obtaining images through the API is only half as high as
        // it could be, since only every other image is actually captured. We work around
        // this by telling the viewport to flush the rendering queue at the end of each
        // drawn frame so that it executes our render request at that point already.
        // Do this only if the main viewport is not being rendered anyway in case there are
        // any adverse performance effects during main rendering.

        // TODO: Validate framerate of sensor data when the NoDisplay setting is turned on.
    }
    else {
        viewport->EngineShowFlags.SetRendering(true);

    }
}

void UAirBlueprintLib::OnBeginPlay()
{
    image_wrapper_module_ = &FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
}

void UAirBlueprintLib::OnEndPlay()
{
    //nothing to do for now
    image_wrapper_module_ = nullptr;
}

IImageWrapperModule* UAirBlueprintLib::getImageWrapperModule()
{
    return image_wrapper_module_;
}

void UAirBlueprintLib::LogMessage(const FString &prefix, const FString &suffix, LogDebugLevel level, float persist_sec)
{
    if (log_messages_hidden_)
        return;


    static TMap<FString, int> loggingKeys;
    static int counter = 1;

    int key = loggingKeys.FindOrAdd(prefix);
    if (key == 0) {
        key = counter++;
        loggingKeys[prefix] = key;
    }

    FColor color;
    switch (level) {
    case LogDebugLevel::Informational:
        color = FColor(147, 231, 237);
        //UE_LOG(LogTemp, Log, TEXT("%s%s"), *prefix, *suffix);
        break;
    case LogDebugLevel::Success:
        color = FColor(156, 237, 147);
        //UE_LOG(LogTemp, Log, TEXT("%s%s"), *prefix, *suffix);
        break;
    case LogDebugLevel::Failure:
        color = FColor(237, 147, 168);
        //UE_LOG(LogAirSim, Error, TEXT("%s%s"), *prefix, *suffix);
        break;
    case LogDebugLevel::Unimportant:
        color = FColor(237, 228, 147);
        //UE_LOG(LogTemp, Verbose, TEXT("%s%s"), *prefix, *suffix);
        break;
    default: color = FColor::Black; break;
    }
    if (GEngine) {
        GEngine->AddOnScreenDebugMessage(key, persist_sec, color, prefix + suffix);
    }
    //GEngine->AddOnScreenDebugMessage(key + 10, 60.0f, color, FString::FromInt(key));
}

void UAirBlueprintLib::setUnrealClockSpeed(const AActor* context, float clock_speed)
{
    UAirBlueprintLib::RunCommandOnGameThread([context, clock_speed]() {
        auto* world_settings = context->GetWorldSettings();
        if (world_settings)
            world_settings->SetTimeDilation(clock_speed);
        else
            LogMessageString("Failed:", "WorldSettings was nullptr", LogDebugLevel::Failure);
    }, true);
}

float UAirBlueprintLib::GetWorldToMetersScale(const AActor* context)
{
    float w2m = 100.f;
    UWorld* w = context->GetWorld();
    if (w != nullptr)
    {
        AWorldSettings* ws = w->GetWorldSettings();
        if (ws != nullptr)
        {
            w2m = ws->WorldToMeters;
        }
    }
    return w2m;
}

template<typename T>
T* UAirBlueprintLib::GetActorComponent(AActor* actor, FString name)
{
    TArray<T*> components;
    actor->GetComponents(components);
    T* found = nullptr;
    for (T* component : components)
    {
        if (component->GetName().Compare(name) == 0) {
            found = component;
            break;
        }
    }
    return found;
}
template UChildActorComponent* UAirBlueprintLib::GetActorComponent(AActor*, FString);
template USceneCaptureComponent2D* UAirBlueprintLib::GetActorComponent(AActor*, FString);
template UStaticMeshComponent* UAirBlueprintLib::GetActorComponent(AActor*, FString);
template URotatingMovementComponent* UAirBlueprintLib::GetActorComponent(AActor*, FString);
template UCameraComponent* UAirBlueprintLib::GetActorComponent(AActor*, FString);

bool UAirBlueprintLib::IsInGameThread()
{
    return ::IsInGameThread();
}

void UAirBlueprintLib::RunCommandOnGameThread(TFunction<void()> InFunction, bool wait, const TStatId InStatId)
{
    if (IsInGameThread())
        InFunction();
    else {
        FGraphEventRef task = FFunctionGraphTask::CreateAndDispatchWhenReady(MoveTemp(InFunction), InStatId, nullptr, ENamedThreads::GameThread);
        if (wait)
            FTaskGraphInterface::Get().WaitUntilTaskCompletes(task);
    }
}

template<>
std::string UAirBlueprintLib::GetMeshName<USkinnedMeshComponent>(USkinnedMeshComponent* mesh)
{
    switch (mesh_naming_method_)
    {
    case msr::airlib::AirSimSettings::SegmentationSetting::MeshNamingMethodType::OwnerName:
        if (mesh->GetOwner())
            return std::string(TCHAR_TO_UTF8(*(mesh->GetOwner()->GetName())));
        else
            return ""; // std::string(TCHAR_TO_UTF8(*(UKismetSystemLibrary::GetDisplayName(mesh))));
    case msr::airlib::AirSimSettings::SegmentationSetting::MeshNamingMethodType::StaticMeshName:
        if (mesh->SkeletalMesh)
            return std::string(TCHAR_TO_UTF8(*(mesh->SkeletalMesh->GetName())));
        else
            return "";
    default:
        return "";
    }
}


std::string UAirBlueprintLib::GetMeshName(ALandscapeProxy* mesh)
{
    return std::string(TCHAR_TO_UTF8(*(mesh->GetName())));
}

void UAirBlueprintLib::InitializeMeshStencilIDs(bool ignore_existing)
{
    for (TObjectIterator<UStaticMeshComponent> comp; comp; ++comp)
    {
        InitializeObjectStencilID(*comp, ignore_existing);
    }
    for (TObjectIterator<USkinnedMeshComponent> comp; comp; ++comp)
    {
        InitializeObjectStencilID(*comp, ignore_existing);
    }
    //for (TObjectIterator<UFoliageType> comp; comp; ++comp)
    //{
    //    InitializeObjectStencilID(*comp);
    //}
    for (TObjectIterator<ALandscapeProxy> comp; comp; ++comp)
    {
        InitializeObjectStencilID(*comp, ignore_existing);
    }
}

bool UAirBlueprintLib::SetMeshStencilID(const std::string& mesh_name, int object_id,
    bool is_name_regex)
{
    std::regex name_regex;

    if (is_name_regex)
        name_regex.assign(mesh_name, std::regex_constants::icase);

    int changes = 0;
    for (TObjectIterator<UStaticMeshComponent> comp; comp; ++comp)
    {
        SetObjectStencilIDIfMatch(*comp, object_id, mesh_name, is_name_regex, name_regex, changes);
    }
    for (TObjectIterator<USkinnedMeshComponent> comp; comp; ++comp)
    {
        SetObjectStencilIDIfMatch(*comp, object_id, mesh_name, is_name_regex, name_regex, changes);
    }
    for (TObjectIterator<ALandscapeProxy> comp; comp; ++comp)
    {
        SetObjectStencilIDIfMatch(*comp, object_id, mesh_name, is_name_regex, name_regex, changes);
    }

    return changes > 0;
}

int UAirBlueprintLib::GetMeshStencilID(const std::string& mesh_name)
{
    FString fmesh_name(mesh_name.c_str());
    for (TObjectIterator<UMeshComponent> comp; comp; ++comp)
    {
        // Access the subclass instance with the * or -> operators.
        UMeshComponent *mesh = *comp;
        if (mesh->GetName() == fmesh_name) {
            return mesh->CustomDepthStencilValue;
        }
    }

    return -1;
}

std::vector<std::string> UAirBlueprintLib::ListMatchingActors(const UObject *context, const std::string& name_regex)
{
    std::vector<std::string> results;
    auto world = context->GetWorld();
    std::regex compiledRegex(name_regex, std::regex::optimize);
    for (TActorIterator<AActor> actorIterator(world); actorIterator; ++actorIterator)
    {
        AActor *actor = *actorIterator;
        auto name = std::string(TCHAR_TO_UTF8(*actor->GetName()));
        bool match = std::regex_match(name, compiledRegex);
        if (match)
            results.push_back(name);
    }
    return results;
}


bool UAirBlueprintLib::HasObstacle(const AActor* actor, const FVector& start, const FVector& end, const AActor* ignore_actor, ECollisionChannel collision_channel)
{
    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceTestByChannel(start, end, collision_channel, trace_params);
}

bool UAirBlueprintLib::GetObstacle(const AActor* actor, const FVector& start, const FVector& end,
    FHitResult& hit, const AActor* ignore_actor, ECollisionChannel collision_channel)
{
    hit = FHitResult(ForceInit);

    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceSingleByChannel(hit, start, end, collision_channel, trace_params);
}

bool UAirBlueprintLib::GetLastObstaclePosition(const AActor* actor, const FVector& start, const FVector& end,
    FHitResult& hit, const AActor* ignore_actor, ECollisionChannel collision_channel)
{
    TArray<FHitResult> hits;

    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    bool has_hit = actor->GetWorld()->LineTraceMultiByChannel(hits, start, end, collision_channel, trace_params);

    if (hits.Num())
        hit = hits.Last(0);

    return has_hit;
}

void UAirBlueprintLib::FollowActor(AActor* follower, const AActor* followee, const FVector& offset, bool fixed_z, float fixed_z_val)
{
    //can we see followee?
    FHitResult hit;
    if (followee == nullptr) {
        return;
    }
    FVector actor_location = followee->GetActorLocation() + FVector(0, 0, 4);
    FVector next_location = actor_location + offset;
    if (fixed_z)
        next_location.Z = fixed_z_val;

    if (GetObstacle(follower, next_location, actor_location, hit, followee)) {
        next_location = hit.ImpactPoint + offset;

        if (GetObstacle(follower, next_location, actor_location, hit, followee)) {
            float next_z = next_location.Z;
            next_location = hit.ImpactPoint - offset;
            next_location.Z = next_z;
        }
    }

    float dist = (follower->GetActorLocation() - next_location).Size();
    float offset_dist = offset.Size();
    float dist_offset = (dist - offset_dist) / offset_dist;
    float lerp_alpha = common_utils::Utils::clip((dist_offset*dist_offset) * 0.01f + 0.01f, 0.0f, 1.0f);
    next_location = FMath::Lerp(follower->GetActorLocation(), next_location, lerp_alpha);
    follower->SetActorLocation(next_location);

    FRotator next_rot = UKismetMathLibrary::FindLookAtRotation(follower->GetActorLocation(), followee->GetActorLocation());
    next_rot = FMath::Lerp(follower->GetActorRotation(), next_rot, 0.5f);
    follower->SetActorRotation(next_rot);
}


int UAirBlueprintLib::RemoveAxisBinding(const FInputAxisKeyMapping& axis, FInputAxisBinding* axis_binding, AActor* actor)
{
    if (axis_binding != nullptr && actor != nullptr) {
        APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

        //remove mapping
        int found_mapping_index = -1, cur_mapping_index = -1;
        for (const auto& axis_arr : controller->PlayerInput->AxisMappings) {
            ++cur_mapping_index;
            if (axis_arr.AxisName == axis.AxisName && axis_arr.Key == axis.Key) {
                found_mapping_index = cur_mapping_index;
                break;
            }
        }
        if (found_mapping_index >= 0)
            controller->PlayerInput->AxisMappings.RemoveAt(found_mapping_index);

        //removing binding
        int found_binding_index = -1, cur_binding_index = -1;
        for (const auto& axis_arr : controller->InputComponent->AxisBindings) {
            ++cur_binding_index;
            if (axis_arr.AxisName == axis_binding->AxisName) {
                found_binding_index = cur_binding_index;
                break;
            }
        }
        if (found_binding_index >= 0)
            controller->InputComponent->AxisBindings.RemoveAt(found_binding_index);

        return found_binding_index;
    }
    else return -1;
}

float UAirBlueprintLib::GetDisplayGamma()
{
    return GEngine->DisplayGamma;
}

void UAirBlueprintLib::EnableInput(AActor* actor)
{
    actor->EnableInput(actor->GetWorld()->GetFirstPlayerController());
}

UObject* UAirBlueprintLib::LoadObject(const std::string& name)
{
    FString str(name.c_str());
    UObject *obj = StaticLoadObject(UObject::StaticClass(), nullptr, *str);
    if (obj == nullptr) {
        std::string msg = "Failed to load asset object - " + name;
        FString fmsg(msg.c_str());
        LogMessage(TEXT("Load: "), fmsg, LogDebugLevel::Failure);
        throw std::invalid_argument(msg);
    }
    return obj;
}

UClass* UAirBlueprintLib::LoadClass(const std::string& name)
{
    FString str(name.c_str());
    UClass *cls = StaticLoadClass(UObject::StaticClass(), nullptr, *str);
    if (cls == nullptr) {
        std::string msg = "Failed to load asset class - " + name;
        FString fmsg(msg.c_str());
        LogMessage(TEXT("Load: "), fmsg, LogDebugLevel::Failure);
        throw std::invalid_argument(msg);
    }
    return cls;
}

void UAirBlueprintLib::CompressImageArray(int32 width, int32 height, const TArray<FColor> &src, TArray<uint8> &dest)
{
    TArray<FColor> MutableSrcData = src;

    // PNGs are saved as RGBA but FColors are stored as BGRA. An option to swap the order upon compression may be added at
    // some point. At the moment, manually swapping Red and Blue
    for (int32 Index = 0; Index < width*height; Index++)
    {
        uint8 TempRed = MutableSrcData[Index].R;
        MutableSrcData[Index].R = MutableSrcData[Index].B;
        MutableSrcData[Index].B = TempRed;
    }

    FObjectThumbnail TempThumbnail;
    TempThumbnail.SetImageSize(width, height);
    TArray<uint8>& ThumbnailByteArray = TempThumbnail.AccessImageData();

    // Copy scaled image into destination thumb
    int32 MemorySize = width*height * sizeof(FColor);
    ThumbnailByteArray.AddUninitialized(MemorySize);
    FMemory::Memcpy(ThumbnailByteArray.GetData(), MutableSrcData.GetData(), MemorySize);

    // Compress data - convert into a .png
    CompressUsingImageWrapper(ThumbnailByteArray, width, height, dest);;
}

bool UAirBlueprintLib::CompressUsingImageWrapper(const TArray<uint8>& uncompressed, const int32 width, const int32 height, TArray<uint8>& compressed)
{
    bool bSucceeded = false;
    compressed.Reset();
    if (uncompressed.Num() > 0)
    {
        IImageWrapperModule* ImageWrapperModule = UAirBlueprintLib::getImageWrapperModule();
        TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule->CreateImageWrapper(EImageFormat::PNG);
        if (ImageWrapper.IsValid() && ImageWrapper->SetRaw(&uncompressed[0], uncompressed.Num(), width, height, ERGBFormat::RGBA, 8))
        {
            compressed = ImageWrapper->GetCompressed();
            bSucceeded = true;
        }
    }

    return bSucceeded;
}
