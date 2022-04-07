// Fill out your copyright notice in the Description page of Project Settings.

#include "ObjectFilter.h"

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/SkeletalMesh.h>
#include <Engine/StaticMesh.h>

FObjectFilter::FObjectFilter()
    : static_mesh_(nullptr)
    , skeletal_mesh_(nullptr)
    , actor_class_(nullptr)
    , component_class_(nullptr)
    , actor_tag_()
    , component_tag_()
    , actor_instance_(nullptr)
{
}

bool FObjectFilter::matchesActor(AActor* actor) const
{
    if (actor_instance_) {
        return actor == actor_instance_;
    }

    if (wildcard_mesh_names_.Num() != 0 && isMatchAnyWildcard(actor->GetName())) {
        return true;
    }

    TInlineComponentArray<UActorComponent*> actor_components;
    actor->GetComponents(actor_components);
    for (UActorComponent* actor_component : actor_components) {
        if (static_mesh_ || wildcard_mesh_names_.Num() != 0) {
            UStaticMeshComponent* static_mesh_component =
                Cast<UStaticMeshComponent>(actor_component);
            if (static_mesh_component) {
                if (static_mesh_ && static_mesh_component->GetStaticMesh() == static_mesh_) {
                    return true;
                }
                if (wildcard_mesh_names_.Num() != 0 &&
                    static_mesh_component->GetStaticMesh() != nullptr &&
                    isMatchAnyWildcard(static_mesh_component->GetStaticMesh()->GetName())) {
                    return true;
                }
            }
        }
        if (skeletal_mesh_ || wildcard_mesh_names_.Num() != 0) {
            USkeletalMeshComponent* skeletal_mesh_component = Cast<USkeletalMeshComponent>(actor_component);
            if (skeletal_mesh_component) {
                if (skeletal_mesh_ &&
                    skeletal_mesh_component->SkeletalMesh == skeletal_mesh_) {
                    return true;
                }
                if (wildcard_mesh_names_.Num() != 0 &&
                    skeletal_mesh_component->SkeletalMesh &&
                    isMatchAnyWildcard(skeletal_mesh_component->SkeletalMesh->GetName())) {
                    return true;
                }
            }
        }
        if (component_class_) {
            if (actor_component->GetClass()->IsChildOf(component_class_)) {
                return true;
            }
        }
        if (!component_tag_.IsNone()) {
            if (actor_component->ComponentHasTag(component_tag_)) {
                return true;
            }
        }
    }

    if (actor_class_) {
        if (actor->GetClass()->IsChildOf(actor_class_)) {
            return true;
        }
    }

    if (!actor_tag_.IsNone()) {
        if (actor->ActorHasTag(actor_tag_)) {
            return true;
        }
    }

    return false;
}

bool FObjectFilter::matchesComponent(UActorComponent* actor_component) const
{
    bool bMatchesStaticMesh = !static_mesh_;
    bool bMatchesSkeletalMesh = !skeletal_mesh_;
    bool bMatchesWildcardMeshName = wildcard_mesh_names_.Num() == 0;
    bool bMatchesActorClass = !actor_class_;
    bool bMatchesActorTag = actor_tag_.IsNone();
    bool bMatchesComponentClass = !component_class_;
    bool bMatchesComponentTag = component_tag_.IsNone();

    if (static_mesh_ || wildcard_mesh_names_.Num() != 0) {
        UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(actor_component);
        if (StaticMeshComponent) {
            if (static_mesh_ && StaticMeshComponent->GetStaticMesh() == static_mesh_) {
                bMatchesStaticMesh = true;
            }
            if (wildcard_mesh_names_.Num() != 0 &&
                StaticMeshComponent->GetStaticMesh()->IsValidLowLevel() &&
                isMatchAnyWildcard(StaticMeshComponent->GetStaticMesh()->GetName())) {
                bMatchesWildcardMeshName = true;
            }
        }
    }
    if (skeletal_mesh_ || wildcard_mesh_names_.Num() != 0) {
        USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(actor_component);
        if (SkeletalMeshComponent) {
            if (skeletal_mesh_ && SkeletalMeshComponent->SkeletalMesh == skeletal_mesh_) {
                bMatchesSkeletalMesh = true;
            }
            if (wildcard_mesh_names_.Num() != 0 &&
                isMatchAnyWildcard(SkeletalMeshComponent->SkeletalMesh->GetName())) {
                bMatchesWildcardMeshName = true;
            }
        }
    }
    if (component_class_) {
        if (actor_component->GetClass()->IsChildOf(component_class_)) {
            bMatchesComponentClass = true;
        }
    }
    if (!component_tag_.IsNone()) {
        if (actor_component->ComponentHasTag(component_tag_)) {
            bMatchesComponentTag = true;
        }
    }

    if (actor_class_) {
        if (actor_component->GetOwner() &&
            actor_component->GetOwner()->GetClass()->IsChildOf(actor_class_)) {
            bMatchesActorClass = true;
        }
    }

    if (!actor_tag_.IsNone()) {
        if (actor_component->GetOwner() &&
            actor_component->GetOwner()->ActorHasTag(actor_tag_)) {
            bMatchesActorTag = true;
        }
    }

    return bMatchesStaticMesh && bMatchesSkeletalMesh &&
           bMatchesWildcardMeshName && bMatchesActorClass && bMatchesActorTag &&
           bMatchesComponentClass && bMatchesComponentClass &&
           bMatchesComponentTag;
}

bool FObjectFilter::isMatchAnyWildcard(FString component_name) const
{
    for (FString wildcard_mesh_name : wildcard_mesh_names_) {
        if (component_name.MatchesWildcard(wildcard_mesh_name)) {
            return true;
        }
    }

    return false;
}

bool operator==(const FObjectFilter& x, const FObjectFilter& y)
{
    // Check pointer equality first for performance
    return x.actor_class_ == y.actor_class_ && x.actor_instance_ == y.actor_instance_ &&
           x.component_class_ == y.component_class_ && x.static_mesh_ == y.static_mesh_ &&
           x.skeletal_mesh_ == y.skeletal_mesh_ && x.actor_tag_ == y.actor_tag_ &&
           x.component_tag_ == y.component_tag_ && x.wildcard_mesh_names_ == y.wildcard_mesh_names_;
}

uint32 GetTypeHash(const FObjectFilter& key)
{
    uint32 key_hash = 0;
    key_hash = HashCombine(key_hash, GetTypeHash(key.actor_class_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.actor_instance_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.actor_tag_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.component_class_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.component_tag_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.skeletal_mesh_));
    key_hash = HashCombine(key_hash, GetTypeHash(key.static_mesh_));
    // KeyHash = HashCombine(KeyHash, GetTypeHash(Key.WildcardMeshNames));

    return key_hash;
}