// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>

#include "ObjectFilter.generated.h"

class UStaticMesh;
class USkeletalMesh;

USTRUCT(BlueprintType)
struct AIRSIM_API FObjectFilter
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, Category = Mesh)
    UStaticMesh* static_mesh_;

    UPROPERTY(EditAnywhere, Category = Mesh)
    USkeletalMesh* skeletal_mesh_;

    UPROPERTY(EditAnywhere, Category = Mesh)
    TArray<FString> wildcard_mesh_names_;

    UPROPERTY(EditAnywhere, Category = Class)
    TSubclassOf<AActor> actor_class_;

    /* Will match for components that have the provided class or Actors that have
     * at least one component with the provided class.*/
    UPROPERTY(EditAnywhere, Category = Class)
    TSubclassOf<UActorComponent> component_class_;

    /* This will Match Actors that have the provided Tag or components if their
     * Owner has the provided Tag */
    UPROPERTY(EditAnywhere, Category = Tag)
    FName actor_tag_;

    UPROPERTY(EditAnywhere, Category = Tag)
    FName component_tag_;

    /* This will only match with if the tested actor is the same as the provided
     * instance.*/
    UPROPERTY(EditInstanceOnly, Category = Instance)
    AActor* actor_instance_;

    FObjectFilter();

    bool matchesActor(AActor* actor) const;

    bool matchesComponent(UActorComponent* actor_component) const;

    bool isMatchAnyWildcard(FString component_name) const;

    friend bool operator==(const FObjectFilter& x, const FObjectFilter& y);

    friend uint32 getTypeHash(const FObjectFilter& key);
};
