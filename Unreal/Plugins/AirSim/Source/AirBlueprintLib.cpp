// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "AirBlueprintLib.h"
#include <exception>
#include "common/common_utils/Utils.hpp"

/*
//TODO: change naming conventions to same as other files?
Naming conventions in this file:
Methods -> CamelCase
parameters -> camel_case
*/


typedef common_utils::Utils Utils;

bool UAirBlueprintLib::log_messages_hidden = false;

void UAirBlueprintLib::LogMessageString(const std::string &prefix, const std::string &suffix, LogDebugLevel level, float persist_sec)
{
    LogMessage(FString(prefix.c_str()), FString(suffix.c_str()), level, persist_sec);
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
    case LogDebugLevel::Informational: color = FColor(5, 5, 100);; break;
    case LogDebugLevel::Success: color = FColor::Green; break;
    case LogDebugLevel::Failure: color = FColor::Red; break;
    case LogDebugLevel::Unimportant: color = FColor::Silver; break;
    default: color = FColor::Black; break;
    }
    GEngine->AddOnScreenDebugMessage(key, persist_sec, color, prefix + suffix);
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

    for (AActor* actor : foundActors) {
        if (actor->GetName().Compare(name) == 0) {
            return static_cast<T*>(actor);
        }
    }

    UAirBlueprintLib::LogMessage(name + TEXT(" Actor not found!"), TEXT(""), LogDebugLevel::Failure);
    return nullptr;
}

template<typename T>
void UAirBlueprintLib::FindAllActor(const UObject* context, TArray<AActor*>& foundActors)
{
    UGameplayStatics::GetAllActorsOfClass(context == nullptr ? GEngine : context, T::StaticClass(), foundActors);
}

bool UAirBlueprintLib::HasObstacle(const AActor* actor, const FVector& start, const FVector& end, const AActor* ignore_actor, ECollisionChannel collison_channel) 
{
    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceTestByChannel(start, end, collison_channel, trace_params);
}

bool UAirBlueprintLib::GetObstacle(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit,  const AActor* ignore_actor, ECollisionChannel collison_channel) 
{
    hit = FHitResult(ForceInit);

    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    return actor->GetWorld()->LineTraceSingleByChannel(hit, start, end, collison_channel, trace_params);
}

bool UAirBlueprintLib::GetLastObstaclePosition(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit, const AActor* ignore_actor, ECollisionChannel collison_channel) 
{
    TArray<FHitResult> hits;

    FCollisionQueryParams trace_params;
    trace_params.AddIgnoredActor(actor);
    if (ignore_actor != nullptr)
        trace_params.AddIgnoredActor(ignore_actor);

    bool has_hit = actor->GetWorld()->LineTraceMultiByChannel(hits, start, end, collison_channel, trace_params);

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
    FVector next_location = followee->GetActorLocation() + offset;
    if (fixed_z)
        next_location.Z = fixed_z_val;

    if (GetLastObstaclePosition(follower, next_location, followee->GetActorLocation(), hit, followee)) {
        next_location = hit.ImpactPoint + offset;

        if (GetLastObstaclePosition(follower, next_location, followee->GetActorLocation(), hit, followee)) {
            float next_z = next_location.Z;
            next_location = hit.ImpactPoint - offset;
            next_location.Z = next_z;
        }
    }

    float dist = (follower->GetActorLocation() - next_location).Size();
    float offset_dist = offset.Size();
    float dist_offset = (dist - offset_dist) / offset_dist;
    float lerp_alpha = Utils::clip((dist_offset*dist_offset) * 0.01f + 0.01f, 0.0f, 1.0f);
    next_location = FMath::Lerp(follower->GetActorLocation(), next_location, lerp_alpha);
    follower->SetActorLocation(next_location);

    FRotator next_rot = UKismetMathLibrary::FindLookAtRotation(follower->GetActorLocation(), followee->GetActorLocation());
    next_rot = FMath::Lerp(follower->GetActorRotation(), next_rot, 0.5f);
    follower->SetActorRotation(next_rot);
}

template<class UserClass>
FInputActionBinding& UAirBlueprintLib::BindActionToKey(const FName action_name, const FKey in_key, UserClass* actor, 
    typename FInputActionHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func,
    bool shift_key, bool control_key, bool alt_key, bool command_key)
{
    FInputActionKeyMapping action(action_name, in_key, shift_key, control_key, alt_key, command_key);
    
    APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

    controller->PlayerInput->AddActionMapping(action);
    return controller->InputComponent->
        BindAction(action_name, IE_Released, actor, func);
}


template<class UserClass>
FInputAxisBinding& UAirBlueprintLib::BindAxisToKey(const FName axis_name, const FKey in_key, UserClass* actor, 
    typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func)
{
    FInputAxisKeyMapping axis(axis_name, in_key);

    APlayerController* controller = actor->GetWorld()->GetFirstPlayerController();

    controller->PlayerInput->AddAxisMapping(axis);
    return controller->InputComponent->
        BindAxis(axis_name, actor, func);
}

void UAirBlueprintLib::EnableInput(AActor* actor)
{
    actor->EnableInput(actor->GetWorld()->GetFirstPlayerController());
}