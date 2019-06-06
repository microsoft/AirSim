// Fill out your copyright notice in the Description page of Project Settings.

#include "BaseIntersection.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"


// Sets default values
ABaseIntersection::ABaseIntersection()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
PrimaryActorTick.bCanEverTick = true;

CollMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CollMesh"));

SetRootComponent(CollMesh);

yellow_light_duration_ = 4.0f;

start_current_traffic_group_delay_ = 6.0f;

current_traffic_group_index_ = 0;

}
void ABaseIntersection::OnConstruction(const FTransform & Transform)
{
	Super::OnConstruction(Transform);
	setTrafficGroupIntersections();


	destroyTextComps();


	// text comps to make it easier to visualize which traffic group which path is
	if (GetWorld() && GetWorld()->WorldType == EWorldType::Editor)
	{
		for (int32 i = 0; i < TrafficGroupArray.Num(); i++)
		{
			for (int32 j = 0; j < TrafficGroupArray[i].CarPaths.Num(); j++)
			{
				if (TrafficGroupArray[i].CarPaths[j] && TrafficGroupArray[i].CarPaths[j]->getCarPathSpline())
				{
					//UE_LOG(LogTemp, Warning, TEXT("%s adding text comp!"), *GetFName().ToString());
					float spline_length = TrafficGroupArray[i].CarPaths[j]->getCarPathSpline()->GetSplineLength();
					FVector end_location = TrafficGroupArray[i].CarPaths[j]->getCarPathSpline()->GetLocationAtDistanceAlongSpline(spline_length, ESplineCoordinateSpace::World);

					// move it up a little bit for more readability
					end_location += FVector(0, 0, 100.0f);

					FString text = FString::FromInt(i);
					//DrawDebugString(UObject* WorldContextObject, const FVector TextLocation, const FString& Text, class AActor* TestBaseActor = NULL, FLinearColor TextColor = FLinearColor::White, float Duration = 0.f);
					//UKismetSystemLibrary::DrawDebugString(GetWorld(), end_location, text, NULL, FLinearColor::White, 10000.0f);

					class UTextRenderComponent* new_text_comp = NewObject<UTextRenderComponent>(this);
					if (new_text_comp)
					{
						new_text_comp->RegisterComponent();

						new_text_comp->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
						new_text_comp->SetWorldLocation(end_location);
						new_text_comp->SetText(text);
						new_text_comp->SetXScale(debug_text_scale);
						new_text_comp->SetYScale(debug_text_scale);

						text_comps_.Add(new_text_comp);
					}
				}
			}
			for (int32 k = 0; k < TrafficGroupArray[i].PedestrianCrossingPaths.Num(); k++)
			{
				if (TrafficGroupArray[i].PedestrianCrossingPaths[k])
				{
					// put it at the middle of the spline
					float spline_length = TrafficGroupArray[i].PedestrianCrossingPaths[k]->getSplineComp()->GetSplineLength() / 2;
					FVector end_location = TrafficGroupArray[i].PedestrianCrossingPaths[k]->getSplineComp()->GetLocationAtDistanceAlongSpline(spline_length, ESplineCoordinateSpace::World);

					// move it up a little bit for more readability
					end_location += FVector(0, 0, 100.0f);

					FString text = FString::FromInt(i);

					class UTextRenderComponent* new_text_comp = NewObject<UTextRenderComponent>(this);
					if (new_text_comp)
					{
						new_text_comp->RegisterComponent();

						new_text_comp->AttachToComponent(RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
						new_text_comp->SetWorldLocation(end_location);
						new_text_comp->SetText(text);
						new_text_comp->SetXScale(debug_text_scale);
						new_text_comp->SetYScale(debug_text_scale);

						new_text_comp->SetTextRenderColor(FColor::Yellow);

						text_comps_.Add(new_text_comp);
					}

				}
			}
		}
	}

}
// Called when the game starts or when spawned
void ABaseIntersection::BeginPlay()
{
	Super::BeginPlay();

	destroyTextComps();

	// if we only have one traffic group, just let it go
	if (TrafficGroupArray.Num() > 1)
	{
		setAllTrafficGroupStatus(ETrafficStatus::TRAFFIC_RED, current_traffic_group_index_);
	}	
	else
	{
		setAllTrafficGroupStatus(ETrafficStatus::TRAFFIC_GREEN, current_traffic_group_index_);
	}
}


void ABaseIntersection::startCurrentTrafficGroup()
{
	setAllTrafficGroupStatus(ETrafficStatus::TRAFFIC_GREEN, current_traffic_group_index_);
}

void ABaseIntersection::lastTrafficGroupRedLightDelayed(float delay)
{
	GetWorld()->GetTimerManager().SetTimer(yellowLight_Timer_, this, &ABaseIntersection::lastTrafficGroupRedLight, yellow_light_duration_, false);
}
void ABaseIntersection::lastTrafficGroupRedLight()
{
	int32 last_traffic_index = current_traffic_group_index_ - 1;

	if (last_traffic_index < 0)
	{
		last_traffic_index = TrafficGroupArray.Num() - 1;
	}

	if (TrafficGroupArray.IsValidIndex(last_traffic_index))
	{
		setAllTrafficGroupStatus(ETrafficStatus::TRAFFIC_RED, last_traffic_index);
	}
}
void ABaseIntersection::setAllTrafficGroupStatus(ETrafficStatus new_traffic_status, int32 index)
{
	if (TrafficGroupArray.IsValidIndex(index))
	{
		for (int32 i = 0; i < TrafficGroupArray[index].TrafficLights.Num(); i++)
		{
			if (new_traffic_status == ETrafficStatus::TRAFFIC_GREEN)
			{
				TrafficGroupArray[index].TrafficLights[i]->TrafficGreenLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_YELLOW)
			{
				TrafficGroupArray[index].TrafficLights[i]->TrafficYellowLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_RED)
			{
				TrafficGroupArray[index].TrafficLights[i]->TrafficRedLight();
			}
		}
		for (int32 i = 0; i < TrafficGroupArray[index].CarPaths.Num(); i++)
		{
			if (new_traffic_status == ETrafficStatus::TRAFFIC_GREEN)
			{
				TrafficGroupArray[index].CarPaths[i]->TrafficGreenLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_YELLOW)
			{
				TrafficGroupArray[index].CarPaths[i]->TrafficYellowLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_RED)
			{
				TrafficGroupArray[index].CarPaths[i]->TrafficRedLight();
			}

		}
		for (int32 i = 0; i < TrafficGroupArray[index].PedestrianCrossingPaths.Num(); i++)
		{
			if (new_traffic_status == ETrafficStatus::TRAFFIC_GREEN)
			{
				TrafficGroupArray[index].PedestrianCrossingPaths[i]->TrafficGreenLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_YELLOW)
			{
				TrafficGroupArray[index].PedestrianCrossingPaths[i]->TrafficYellowLight();
			}
			if (new_traffic_status == ETrafficStatus::TRAFFIC_RED)
			{
				TrafficGroupArray[index].PedestrianCrossingPaths[i]->TrafficRedLight();
			}
		}
	}
}
void ABaseIntersection::initAllIntersections()
{
	TArray<AActor*> intersections;

	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABaseIntersection::StaticClass(), intersections);

	for (int32 i = 0; i < intersections.Num(); i++)
	{
		ABaseIntersection* cast_intersection = Cast<ABaseIntersection>(intersections[i]);
		if (cast_intersection)
		{
			cast_intersection->setTrafficGroupIntersections();
		}
	}
}
void ABaseIntersection::setTrafficGroupIntersections()
{
	for (int32 i = 0; i < TrafficGroupArray.Num(); i++)
	{
		for (int32 j = 0; j < TrafficGroupArray[i].CarPaths.Num(); j++)
		{
			if (TrafficGroupArray[i].CarPaths[j])
			{
				TrafficGroupArray[i].CarPaths[j]->setIntersection(this);
			}
		}
	}
}
void ABaseIntersection::destroyTextComps()
{
	for (int32 i = 0; i < text_comps_.Num(); i++)
	{
		if (text_comps_[i])
		{
			text_comps_[i]->DestroyComponent();
			UE_LOG(LogTemp, Warning, TEXT("%s destroying text comp!"), *GetFName().ToString());

			if (text_comps_[i])
			{
				text_comps_[i]->SetActive(false);
				text_comps_[i]->UnregisterComponent();
			}
		}
		text_comps_.Empty();
	}

	TArray<UActorComponent*> text_comps_deleteme = GetComponentsByClass(UTextRenderComponent::StaticClass());
	for (int32 i = 0; i < text_comps_deleteme.Num(); i++)
	{
		if (text_comps_deleteme[i])
		{
			text_comps_deleteme[i]->DestroyComponent();
			if (text_comps_deleteme[i])
			{
				text_comps_deleteme[i]->SetActive(false);
				text_comps_deleteme[i]->UnregisterComponent();
			}
		}
	}
}
// Called every frame
void ABaseIntersection::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// cycle traffic groups if we have more than one
	if (GetWorld() && TrafficGroupArray.Num() > 1)
	{
		// if time seconds is greater than next traffic group change time
		if (UGameplayStatics::GetTimeSeconds(GetWorld()) >= next_traffic_change_time_)
		{
			if (TrafficGroupArray.IsValidIndex(current_traffic_group_index_))
			{
				next_traffic_change_time_ = TrafficGroupArray[current_traffic_group_index_].Duration + UGameplayStatics::GetTimeSeconds(GetWorld());

				// current group yellow light, then red after a delay
				setAllTrafficGroupStatus(ETrafficStatus::TRAFFIC_YELLOW, current_traffic_group_index_);

				// increment current traffic index
				current_traffic_group_index_++;
				if (current_traffic_group_index_ >= TrafficGroupArray.Num())
				{
					current_traffic_group_index_ = 0;
				}

				// THEN set the last group to be red, after a delay
				lastTrafficGroupRedLightDelayed(yellow_light_duration_);

				//start current traffic group delayed
				//startCurrentTrafficGroupDelayed(start_current_traffic_group_delay_);
				GetWorld()->GetTimerManager().SetTimer(currentTrafficGroup_Timer_, this, &ABaseIntersection::startCurrentTrafficGroup, start_current_traffic_group_delay_, false);
			}
		}
	}

	if (TrafficGroupArray.Num() > 0 && TrafficGroupArray.IsValidIndex(current_traffic_group_index_))
	{
		for (int32 i = 0; i < TrafficGroupArray[current_traffic_group_index_].CarPaths.Num(); i++)
		{
			ABaseCarPath* carpath = TrafficGroupArray[current_traffic_group_index_].CarPaths[i];
			if (carpath && carpath->GetTrafficLightStatus() == ETrafficStatus::TRAFFIC_GREEN)
			{
				carpath->nextWaitingVehicleStartMovingOnConnectingSpline();
			}
		}
	}
}

