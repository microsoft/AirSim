// Fill out your copyright notice in the Description page of Project Settings.

#include "BaseCarPathCollSphereComponent.h"
#include "Kismet/GameplayStatics.h"
//#include "GameFramework\Gamestate.h"


/*UBaseCarPathCollSphereComponent::UBaseCarPathCollSphereComponent(const FObjectInitializer& ObjectInitializer):Super(ObjectInitializer)
{
	SphereRadius = 32.0f;
	ShapeColor = FColor(255, 0, 0, 255);

	bUseEditorCompositing = true;
}*/

UBaseCarPathCollSphereComponent::UBaseCarPathCollSphereComponent(const FObjectInitializer& ObjectInitializer) :Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;

	PrimaryComponentTick.TickInterval = 0.25f;

	//bDrawDebug = true;

	//SetVisibility(true, true);
	//SetHiddenInGame(false);

}
/*UBaseCarPathCollSphereComponent::UBaseCarPathCollSphereComponent()
{

}*/
void UBaseCarPathCollSphereComponent::enableCollision(float AutoDisableDelay)
{
	// only set the timer if we were disabled and we are now enabling it
	if (GetWorld())
	{
		if (GetCollisionEnabled() == ECollisionEnabled::NoCollision)
		{
			enabled_time = UGameplayStatics::GetTimeSeconds(GetWorld());
		}
		DisableTime = UGameplayStatics::GetTimeSeconds(GetWorld()) + AutoDisableDelay;

		if (bDrawDebug)
		{
			FLinearColor TestColor;
			TestColor.R = 255;
			TestColor.G = 0;
			TestColor.B = 0;
			UKismetSystemLibrary::DrawDebugPoint(GetWorld(), GetComponentLocation(), 25.0f, TestColor, 0.25f);
		}
	}
	SetCollisionEnabled(ECollisionEnabled::QueryOnly);
}
void UBaseCarPathCollSphereComponent::disableCollision()
{
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
}
void UBaseCarPathCollSphereComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{

	/*if (CVarDrawCarCollisionPrediction.GetValueOnGameThread() >= 1)
	{

	}*/
	if (GetWorld())
	{
		if (DisableTime > UGameplayStatics::GetTimeSeconds(GetWorld()) )
		{
			disableCollision();
		}


		if (bDrawDebug)
		{
			if (GetCollisionEnabled() == ECollisionEnabled::QueryOnly)
			{
				FLinearColor TestColor;
				TestColor.R = 255;
				TestColor.G = 0;
				TestColor.B = 0;
				UKismetSystemLibrary::DrawDebugPoint(GetWorld(), GetComponentLocation(), 25.0f, TestColor, 0.25f);
			}
			else
			{
				FLinearColor TestColor;
				TestColor.R = 0;
				TestColor.G = 255;
				TestColor.B = 0;
				UKismetSystemLibrary::DrawDebugPoint(GetWorld(), GetComponentLocation(), 10.0f, TestColor, 0.25f);
			}
		}

	}
}