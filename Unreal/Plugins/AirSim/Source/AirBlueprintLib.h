// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerInput.h"
#include <string>
#include <regex>
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "Components/MeshComponent.h"
#include "LandscapeProxy.h"
#include "AirSim.h"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.generated.h"


UENUM(BlueprintType)
enum class LogDebugLevel : uint8 {
    Informational UMETA(DisplayName="Informational"),
    Success UMETA(DisplayName = "Success"),
    Failure UMETA(DisplayName = "Failure"),
    Unimportant  UMETA(DisplayName = "Unimportant")
};

/**
 * 
 */
UCLASS()
class UAirBlueprintLib : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    static void OnBeginPlay();
    static void OnEndPlay();
    static void LogMessageString(const std::string &prefix, const std::string &suffix, LogDebugLevel level, float persist_sec = 60);
    UFUNCTION(BlueprintCallable, Category = "Utils")
    static void LogMessage(const FString &prefix, const FString &suffix, LogDebugLevel level, float persist_sec = 60);
    static float GetWorldToMetersScale(const AActor* context);

    template<typename T>
    static T* GetActorComponent(AActor* actor, FString name);
    template<typename T>
    static T* FindActor(const UObject* context, FString name);
    template<typename T>
    static void FindAllActor(const UObject* context, TArray<AActor*>& foundActors);
    static bool HasObstacle(const AActor* actor, const FVector& start, const FVector& end, const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static bool GetObstacle(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit, const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static bool GetLastObstaclePosition(const AActor* actor, const FVector& start, const FVector& end, FHitResult& hit, const AActor* ignore_actor = nullptr, ECollisionChannel collision_channel = ECC_Visibility);
    static void FollowActor(AActor* follower, const AActor* followee, const FVector& offset, bool fixed_z = false, float fixed_z_val = 2.0f);

    static bool SetMeshStencilID(const std::string& mesh_name, int object_id,
        bool is_name_regex = false);
    static int GetMeshStencilID(const std::string& mesh_name);
    static void InitializeMeshStencilIDs(bool ignore_existing);

    static bool IsInGameThread();
    
    template<class T>
    static std::string GetMeshName(T* mesh);
    static std::string GetMeshName(ALandscapeProxy* mesh);


    template<class UserClass>
    static FInputActionBinding& BindActionToKey(const FName action_name, const FKey in_key, UserClass* actor,
        typename FInputActionHandlerSignature::TUObjectMethodDelegate< UserClass >::FMethodPtr func, bool on_press_or_release = false,
        bool shift_key = false, bool control_key = false, bool alt_key = false, bool command_key = false);

    template<class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FName axis_name, const FKey in_key, AActor* actor, UserClass* obj,
        typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func);
    template<class UserClass>
    static FInputAxisBinding& BindAxisToKey(const FInputAxisKeyMapping& axis, AActor* actor, UserClass* obj,
        typename FInputAxisHandlerSignature::TUObjectMethodDelegate<UserClass>::FMethodPtr func);

    static int RemoveAxisBinding(const FInputAxisKeyMapping& axis, FInputAxisBinding* axis_binding, AActor* actor);

    static void EnableInput(AActor* actor);

    static void RunCommandOnGameThread(TFunction<void()> InFunction, bool wait = false, const TStatId InStatId = TStatId());

    static float GetDisplayGamma();

    static EAppReturnType::Type ShowMessage(EAppMsgType::Type MessageType, const std::string& message, const std::string& title);

    static bool getLogMessagesHidden()
    {
        return log_messages_hidden;
    }
    static void setLogMessagesHidden(bool is_hidden)
    {
        log_messages_hidden = is_hidden;
    }
    static void SetMeshNamingMethod(msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType method)
    {
        mesh_naming_method = method;
    }

    static void enableWorldRendering(AActor* context, bool enable);
    static void enableViewportRendering(AActor* context, bool enable);
    static void setSimulatePhysics(AActor* actor, bool simulate_physics);
    static void resetSimulatePhysics(AActor* actor);
    static std::vector<UPrimitiveComponent*> getPhysicsComponents(AActor* actor);

    static UObject* LoadObject(const std::string& name);

private:
    template<typename T>
    static void InitializeObjectStencilID(T* obj, bool ignore_existing = true);


    template<typename T>
    static void SetObjectStencilIDIfMatch(T* mesh, int object_id, 
        const std::string& mesh_name, bool is_name_regex, const std::regex& name_regex, int& changes);

    template<typename T>
    static void SetObjectStencilID(T* mesh, int object_id);
    static void SetObjectStencilID(ALandscapeProxy* mesh, int object_id);


private:
    static bool log_messages_hidden;
    //FViewPort doesn't expose this field so we are doing dirty work around by maintaining count by ourselves
    static uint32_t FlushOnDrawCount;
    static msr::airlib::AirSimSettings::SegmentationSettings::MeshNamingMethodType mesh_naming_method;
};

