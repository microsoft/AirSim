// Fill out your copyright notice in the Description page of Project Settings.

#include "ObjectFilter.h"

#include <Components/SkeletalMeshComponent.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/SkeletalMesh.h>
#include <Engine/StaticMesh.h>

FObjectFilter::FObjectFilter()
	: StaticMesh(nullptr)
	, SkeletalMesh(nullptr)
	, ActorClass(nullptr)
	, ComponentClass(nullptr)
	, ActorTag()
	, ComponentTag()
	, ActorInstance(nullptr)
{

}

bool FObjectFilter::MatchesActor(AActor* Actor) const 
{
	if (ActorInstance) 
	{
		return Actor == ActorInstance;
	}
	
	bool bMatchesStaticMesh = false;
	bool bMatchesSkeletalMesh = false;
	bool bMatchesWildcardMeshName = false;
	bool bMatchesActorClass = false;
	bool bMatchesActorTag = false;
	bool bMatchesComponentClass = false;
	bool bMatchesComponentTag = false;

	TInlineComponentArray<UActorComponent*> ActorComponents;
	Actor->GetComponents(ActorComponents);
	for (UActorComponent* ActorComponent : ActorComponents) 
	{
		if (StaticMesh || !WildcardMeshNames.Num() == 0) 
		{
			UStaticMeshComponent* StaticMeshComponent =
				Cast<UStaticMeshComponent>(ActorComponent);
			if (StaticMeshComponent) {
				if (StaticMesh && StaticMeshComponent->GetStaticMesh() == StaticMesh) 
				{
					//bMatchesStaticMesh = true;
					return true;
				}
				if (!WildcardMeshNames.Num() == 0 &&
					IsMatchAnyWildcard(StaticMeshComponent->GetStaticMesh()->GetName()))
				{
					//bMatchesWildcardMeshName = true;
					return true;
				}
			}
		}
		if (SkeletalMesh || !WildcardMeshNames.Num() == 0) 
		{
			USkeletalMeshComponent* SkeletalMeshComponent =	Cast<USkeletalMeshComponent>(ActorComponent);
			if (SkeletalMeshComponent) 
			{
				if (SkeletalMesh &&
					SkeletalMeshComponent->SkeletalMesh == SkeletalMesh) 
				{
					//bMatchesSkeletalMesh = true;
					return true;
				}
				if (!WildcardMeshNames.Num() == 0 &&
					IsMatchAnyWildcard(SkeletalMeshComponent->SkeletalMesh->GetName()))
				{
					//bMatchesWildcardMeshName = true;
					return true;
				}
			}
		}
		if (ComponentClass) 
		{
			if (ActorComponent->GetClass()->IsChildOf(ComponentClass)) 
			{
			//	bMatchesComponentClass = true;
				return true;
			}
		}
		if (!ComponentTag.IsNone()) 
		{
			if (ActorComponent->ComponentHasTag(ComponentTag)) 
			{
				//bMatchesComponentTag = true;
				return true;
			}
		}
	}

	if (ActorClass) 
	{
		if (Actor->GetClass()->IsChildOf(ActorClass)) 
		{
			//bMatchesActorClass = true;
			return true;
		}
	}

	if (!ActorTag.IsNone()) 
	{
		if (Actor->ActorHasTag(ActorTag)) 
		{
		//	bMatchesActorTag = true;
			return true;
		}
	}

	return bMatchesStaticMesh || bMatchesSkeletalMesh ||
		bMatchesWildcardMeshName || bMatchesActorClass || bMatchesActorTag ||
		bMatchesComponentClass || bMatchesComponentClass ||
		bMatchesComponentTag;
}

bool FObjectFilter::MatchesComponent(UActorComponent* ActorComponent) const 
{
	bool bMatchesStaticMesh = !StaticMesh;
	bool bMatchesSkeletalMesh = !SkeletalMesh;
	bool bMatchesWildcardMeshName = WildcardMeshNames.Num() == 0;
	bool bMatchesActorClass = !ActorClass;
	bool bMatchesActorTag = ActorTag.IsNone();
	bool bMatchesComponentClass = !ComponentClass;
	bool bMatchesComponentTag = ComponentTag.IsNone();

	if (StaticMesh || !WildcardMeshNames.Num() == 0)
	{
		UStaticMeshComponent* StaticMeshComponent =	Cast<UStaticMeshComponent>(ActorComponent);
		if (StaticMeshComponent) 
		{
			if (StaticMesh && StaticMeshComponent->GetStaticMesh() == StaticMesh)
			{
				bMatchesStaticMesh = true;
			}
			if (!WildcardMeshNames.Num() == 0 &&
				StaticMeshComponent->GetStaticMesh()->IsValidLowLevel() &&
				IsMatchAnyWildcard(StaticMeshComponent->GetStaticMesh()->GetName()))
			{
				bMatchesWildcardMeshName = true;
			}
		}
	}
	if (SkeletalMesh || !WildcardMeshNames.Num() == 0) 
	{
		USkeletalMeshComponent* SkeletalMeshComponent =	Cast<USkeletalMeshComponent>(ActorComponent);
		if (SkeletalMeshComponent)
		{
			if (SkeletalMesh && SkeletalMeshComponent->SkeletalMesh == SkeletalMesh)
			{
				bMatchesSkeletalMesh = true;
			}
			if (!WildcardMeshNames.Num() == 0 &&
				IsMatchAnyWildcard(SkeletalMeshComponent->SkeletalMesh->GetName()))
			{
				bMatchesWildcardMeshName = true;
			}
		}
	}
	if (ComponentClass)
	{
		if (ActorComponent->GetClass()->IsChildOf(ComponentClass))
		{
			bMatchesComponentClass = true;
		}
	}
	if (!ComponentTag.IsNone())
	{
		if (ActorComponent->ComponentHasTag(ComponentTag))
		{
			bMatchesComponentTag = true;
		}
	}

	if (ActorClass)
	{
		if (ActorComponent->GetOwner() &&
			ActorComponent->GetOwner()->GetClass()->IsChildOf(ActorClass))
		{
			bMatchesActorClass = true;
		}
	}

	if (!ActorTag.IsNone())
	{
		if (ActorComponent->GetOwner() &&
			ActorComponent->GetOwner()->ActorHasTag(ActorTag))
		{
			bMatchesActorTag = true;
		}
	}

	return bMatchesStaticMesh && bMatchesSkeletalMesh &&
		bMatchesWildcardMeshName && bMatchesActorClass && bMatchesActorTag &&
		bMatchesComponentClass && bMatchesComponentClass &&
		bMatchesComponentTag;
}

bool FObjectFilter::IsMatchAnyWildcard(FString ComponentName) const
{
	for (FString WildcardMeshName : WildcardMeshNames)
	{
		if (ComponentName.MatchesWildcard(WildcardMeshName))
		{
			return true;
		}
	}

	return false;
}

bool operator==(const FObjectFilter& X, const FObjectFilter& Y) 
{
	// Check pointer eqaulity first for performance
	return X.ActorClass == Y.ActorClass && X.ActorInstance == Y.ActorInstance &&
		X.ComponentClass == Y.ComponentClass && X.StaticMesh == Y.StaticMesh &&
		X.SkeletalMesh == Y.SkeletalMesh && X.ActorTag == Y.ActorTag &&
		X.ActorTag == Y.ActorTag && X.ComponentTag == Y.ComponentTag &&
		X.WildcardMeshNames == Y.WildcardMeshNames;
}

uint32 GetTypeHash(const FObjectFilter& Key) 
{
	uint32 KeyHash = 0;
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.ActorClass));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.ActorInstance));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.ActorTag));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.ComponentClass));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.ComponentTag));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.SkeletalMesh));
	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.StaticMesh));
//	KeyHash = HashCombine(KeyHash, GetTypeHash(Key.WildcardMeshNames));

	return KeyHash;
}