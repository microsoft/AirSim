// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirBlueprintLib.h"
#include "GameFramework/WorldSettings.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkinnedMeshComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/RotatingMovementComponent.h"
#include <exception>
#include "common/common_utils/Utils.hpp"
#include "Components/StaticMeshComponent.h"
#include "EngineUtils.h"
#include "Runtime/Landscape/Classes/LandscapeComponent.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "UObjectIterator.h"
//#include "Runtime/Foliage/Public/FoliageType.h"
#include "Kismet/KismetStringLibrary.h"
#include "MessageDialog.h"
#include "Engine/LocalPlayer.h"
#include "Engine/SkeletalMesh.h"
#include "Slate/SceneViewport.h"
#include "Engine/Engine.h"

/*
//TODO: change naming conventions to same as other files?
Naming conventions in this file:
Methods -> CamelCase
parameters -> camel_case
*/

bool UAirBlueprintLib::log_messages_hidden = false;
uint32_t UAirBlueprintLib::FlushOnDrawCount = 0;
msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType UAirBlueprintLib::mesh_naming_method =
    msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType::OwnerName;

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
        //HACK: FViewPort doesn't expose this field so we are doing dirty work around by maintaining count by ourselves
        if (FlushOnDrawCount == 0)
            viewport->GetGameViewport()->IncrementFlushOnDraw();
    }
    else {
        viewport->EngineShowFlags.SetRendering(true);

        //HACK: FViewPort doesn't expose this field so we are doing dirty work around by maintaining count by ourselves
        if (FlushOnDrawCount > 0)
            viewport->GetGameViewport()->DecrementFlushOnDraw();
    }
}

void UAirBlueprintLib::OnBeginPlay()
{
    FlushOnDrawCount = 0;
}

void UAirBlueprintLib::OnEndPlay()
{
    //nothing to do for now
}

void UAirBlueprintLib::LogMessage(const FString &prefix, const FString &suffix, LogDebugLevel level, float persist_sec)
{
    if (log_messages_hidden)
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
        UE_LOG(LogAirSim, Log, TEXT("%s%s"), *prefix, *suffix);
        break;
    case LogDebugLevel::Success: 
        color = FColor(156, 237, 147); 
        UE_LOG(LogAirSim, Log, TEXT("%s%s"), *prefix, *suffix);
        break;
    case LogDebugLevel::Failure: 
        color = FColor(237, 147, 168);
        UE_LOG(LogAirSim, Error, TEXT("%s%s"), *prefix, *suffix); 
        break;
    case LogDebugLevel::Unimportant: 
        color = FColor(237, 228, 147);
        UE_LOG(LogAirSim, Verbose, TEXT("%s%s"), *prefix, *suffix); 
        break;
    default: color = FColor::Black; break;
    }
    if (GEngine) {
        GEngine->AddOnScreenDebugMessage(key, persist_sec, color, prefix + suffix);
    }
    //GEngine->AddOnScreenDebugMessage(key + 10, 60.0f, color, FString::FromInt(key));
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

template<typename T>
T* UAirBlueprintLib::FindActor(const UObject* context, FString name)
{
    TArray<AActor*> foundActors;
    FindAllActor<T>(context, foundActors);
    FName name_n = FName(*name);

    for (AActor* actor : foundActors) {
        if (actor->ActorHasTag(name_n) || actor->GetName().Compare(name) == 0) {
            return static_cast<T*>(actor);
        }
    }

    //UAirBlueprintLib::LogMessage(name + TEXT(" Actor not found!"), TEXT(""), LogDebugLevel::Failure);
    return nullptr;
}

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


template<typename T>
void UAirBlueprintLib::FindAllActor(const UObject* context, TArray<AActor*>& foundActors)
{
    UGameplayStatics::GetAllActorsOfClass(context == nullptr ? GEngine : context, T::StaticClass(), foundActors);
}

template<typename T>
void UAirBlueprintLib::InitializeObjectStencilID(T* mesh, bool ignore_existing)
{
    std::string mesh_name = common_utils::Utils::toLower(GetMeshName(mesh));
    if (mesh_name == "" || common_utils::Utils::startsWith(mesh_name, "default_")) {
        //common_utils::Utils::DebugBreak();
        return;
    }
    FString name(mesh_name.c_str());
    int hash = 5;
    for (int idx = 0; idx < name.Len(); ++idx) {
        auto char_num = UKismetStringLibrary::GetCharacterAsNumber(name, idx);
        if (char_num < 97)
            continue; //numerics and other punctuations
        hash += char_num;
    }
    if (ignore_existing || mesh->CustomDepthStencilValue == 0) { //if value is already set then don't bother
        SetObjectStencilID(mesh, hash % 256);
    }
}

template<typename T>
void UAirBlueprintLib::SetObjectStencilID(T* mesh, int object_id)
{
    if (object_id < 0)
    {
        mesh->SetRenderCustomDepth(false);
    }
    else
    {
        mesh->SetCustomDepthStencilValue(object_id);
        mesh->SetRenderCustomDepth(true);
    }
    //mesh->SetVisibility(false);
    //mesh->SetVisibility(true);
}

void UAirBlueprintLib::SetObjectStencilID(ALandscapeProxy* mesh, int object_id)
{
    if (object_id < 0)
    {
        mesh->bRenderCustomDepth = false;
    }
    else
    {
        mesh->CustomDepthStencilValue = object_id;
        mesh->bRenderCustomDepth = true;
    }

    // Explicitly set the custom depth state on the components so the
    // render state is marked dirty and the update actually takes effect
    // immediately.
    for (ULandscapeComponent* comp : mesh->LandscapeComponents)
    {
        if (object_id < 0)
        {
            comp->SetRenderCustomDepth(false);
        }
        else
        {
            comp->SetCustomDepthStencilValue(object_id);
            comp->SetRenderCustomDepth(true);
        }
    }
}

template<class T>
std::string UAirBlueprintLib::GetMeshName(T* mesh)
{
    switch(mesh_naming_method)
    {
    case msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType::OwnerName:
        if (mesh->GetOwner())
            return std::string(TCHAR_TO_UTF8(*(mesh->GetOwner()->GetName())));
        else
            return ""; // std::string(TCHAR_TO_UTF8(*(UKismetSystemLibrary::GetDisplayName(mesh))));
    case msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType::StaticMeshName:
        if (mesh->GetStaticMesh())
            return std::string(TCHAR_TO_UTF8(*(mesh->GetStaticMesh()->GetName())));
        else
            return "";
    default:
        return "";
    }
}

template<>
std::string UAirBlueprintLib::GetMeshName<USkinnedMeshComponent>(USkinnedMeshComponent* mesh)
{
  switch(mesh_naming_method)
  {
    case msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType::OwnerName:
      if (mesh->GetOwner())
        return std::string(TCHAR_TO_UTF8(*(mesh->GetOwner()->GetName())));
      else
        return ""; // std::string(TCHAR_TO_UTF8(*(UKismetSystemLibrary::GetDisplayName(mesh))));
    case msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType::StaticMeshName:
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

template<typename T>
void UAirBlueprintLib::SetObjectStencilIDIfMatch(T* mesh, int object_id, const std::string& mesh_name, bool is_name_regex, 
    const std::regex& name_regex, int& changes)
{
    std::string comp_mesh_name = GetMeshName(mesh);
    if (comp_mesh_name == "")
        return;
    bool is_match = (!is_name_regex && (comp_mesh_name == mesh_name))
        || (is_name_regex && std::regex_match(comp_mesh_name, name_regex));
    if (is_match) {
        ++changes;
        SetObjectStencilID(mesh, object_id);
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

bool UAirBlueprintLib::HasObstacle(const AActor* actor, const FVector& start, const FVector& end, const AActor* ignore_actor, ECollisionChannel collision_channel) 
{
    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceTestByChannel(start, end, collision_channel, trace_params);
}

bool UAirBlueprintLib::GetObstacle(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit,  const AActor* ignore_actor, ECollisionChannel collision_channel) 
{
    hit = FHitResult(ForceInit);

    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceSingleByChannel(hit, start, end, collision_channel, trace_params);
}

bool UAirBlueprintLib::GetLastObstaclePosition(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit, const AActor* ignore_actor, ECollisionChannel collision_channel) 
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

template<class UserClass>
FInputActionBinding& UAirBlueprintLib::BindActionToKey(const FName action_name, const FKey in_key, UserClass* actor, 
    typename FInputActionHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func, bool on_press_or_release,
    bool shift_key, bool control_key, bool alt_key, bool command_key)
{
    FInputActionKeyMapping action(action_name, in_key, shift_key, control_key, alt_key, command_key);
    
    APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

    controller->PlayerInput->AddActionMapping(action);
    return controller->InputComponent->
        BindAction(action_name, on_press_or_release ? IE_Pressed : IE_Released, actor, func);
}


template<class UserClass>
FInputAxisBinding& UAirBlueprintLib::BindAxisToKey(const FName axis_name, const FKey in_key, AActor* actor, UserClass* obj, 
    typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func)
{
    FInputAxisKeyMapping axis(axis_name, in_key);

    return UAirBlueprintLib::BindAxisToKey(axis, actor, obj, func);
}

template<class UserClass>
FInputAxisBinding& UAirBlueprintLib::BindAxisToKey(const FInputAxisKeyMapping& axis, AActor* actor, UserClass* obj,
    typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func)
{
    APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

    controller->PlayerInput->AddAxisMapping(axis);
    return controller->InputComponent->
        BindAxis(axis.AxisName, obj, func);
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
        std::string msg = "Failed to load asset - " + name;
        FString fmsg(msg.c_str());
        LogMessage(TEXT("Load: "), fmsg, LogDebugLevel::Failure);
        throw std::invalid_argument(msg);
    }
    return obj;
}