// Fill out your copyright notice in the Description page of Project Settings.
#include "SplinePathInfoLib.h"
#include "Engine/CollisionProfile.h"

#include "CollisionQueryParams.h"

#include "SplineViolationChecker.h"


FSplinePathInfo USplinePathInfoLib::getCurrentSplineInfo(UWorld* world, FVector location, FRotator rotation, float trace_size)
{
	FSplinePathInfo SplineInfoToReturn;

	SplineInfoToReturn.spline_meta_data_.is_school_zone = false;
	SplineInfoToReturn.spline_meta_data_.speed_limit = -1.0f;

	TArray<FHitResult> OutHits;

	if (world != nullptr)
	{
		// this trace works better, more likely to get intersection actors
		const TArray<AActor*> ActorsToIgnore;
		FCollisionQueryParams Params = FCollisionQueryParams::DefaultQueryParam;

		Params.bReturnPhysicalMaterial = true;
		Params.bTraceAsyncScene = true;
		Params.AddIgnoredActors(ActorsToIgnore);

		world->SweepMultiByChannel(OutHits, location, location, FQuat::Identity, COLLISION_TRAFFICTRACE, FCollisionShape::MakeSphere(trace_size), Params);
	}
	else
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::White, TEXT("WORLD IS NULL"));
		UE_LOG(LogTemp, Warning, TEXT("WORLD IS NULL") );
	}
	float ClosestSplinePoint = 99999999999;
	ABaseCarPath* ClosestCarPath = nullptr;


	for (int32 i = 0; i < OutHits.Num(); i++)
	{
		ABaseIntersection* TestIntersection = Cast<ABaseIntersection>(OutHits[i].GetActor());
		if (TestIntersection)
		{
			SplineInfoToReturn.is_in_intersection = true;
		}

		ABaseCarPath* TestCarPath = Cast<ABaseCarPath>(OutHits[i].GetActor());
		if (TestCarPath)
		{
			if (TestCarPath->getCarPathSpline())
			{
				SplineInfoToReturn.all_spline_comps_.Add(TestCarPath->getCarPathSpline());
			}

			FVector CurrentSplineClosestLocation = TestCarPath->getCarPathSpline()->FindLocationClosestToWorldLocation(location, ESplineCoordinateSpace::World);
			float CurrentDist = FMath::Abs((CurrentSplineClosestLocation - location).Size());

			if (CurrentDist <= ClosestSplinePoint)
			{
				ClosestSplinePoint = CurrentDist;
				ClosestCarPath = TestCarPath;
			}
		}
	}
	// dist to closest spline point
	SplineInfoToReturn.dist_from_lane_ = ClosestSplinePoint;
	if (ClosestCarPath && ClosestCarPath->getCarPathSpline())
	{
		SplineInfoToReturn.car_path_actor_ = ClosestCarPath;
		SplineInfoToReturn.spline_comp_ = ClosestCarPath->getCarPathSpline();

		SplineInfoToReturn.next_traffic_light_status_ = ClosestCarPath->Execute_GetTrafficLightStatus(ClosestCarPath);

		// get metadata
		bool bShouldLoop = true;
		int32 LocalSplineIndex = 0;
		float CurrentDistOnSplinePoint = getDistOnSplineClosestToLocation(location, ClosestCarPath->getCarPathSpline());
		// loop until we have a point that has a greater dist than our current dist on spline
		while (bShouldLoop)
		{
			if (ClosestCarPath->getCarPathSpline()->GetDistanceAlongSplineAtSplinePoint(LocalSplineIndex) <= CurrentDistOnSplinePoint 
				&& LocalSplineIndex <= ClosestCarPath->getCarPathSpline()->GetNumberOfSplinePoints() )
			{
				LocalSplineIndex++;
			}
			else
			{
				bShouldLoop = false;
			}
		}
		if (ClosestCarPath->getCarPathSpline()->spline_metadata_.IsValidIndex(LocalSplineIndex - 1))
		{
			SplineInfoToReturn.spline_meta_data_ = ClosestCarPath->getCarPathSpline()->spline_metadata_[LocalSplineIndex - 1];
		}

		// this just gets the direction of closest point on spline
		SplineInfoToReturn.spline_direction_ = ClosestCarPath->getCarPathSpline()->FindTransformClosestToWorldLocation(location, ESplineCoordinateSpace::World).Rotator();

		ABaseIntersection* next_intersection = ClosestCarPath->getIntersection();
		if (next_intersection)
		{
			FVector closest_coll_point_location;
			next_intersection->CollMesh->GetClosestPointOnCollision(location, closest_coll_point_location);
			SplineInfoToReturn.next_traffic_light_dist_ = (closest_coll_point_location - location).Size();
			SplineInfoToReturn.next_traffic_light_loc_ = closest_coll_point_location;
		}
	}
	
	return SplineInfoToReturn;

}
float USplinePathInfoLib::getDistOnSplineClosestToLocation(FVector location, UAirSimSplineComp* spline_comp)
{
	if (spline_comp == nullptr)
	{
		return 0.0f;
	}
	float DistAtTruncatedLoc = spline_comp->GetDistanceAlongSplineAtSplinePoint(FMath::TruncToInt(spline_comp->FindInputKeyClosestToWorldLocation(location)));

	float DistAtTruncatedLocPlusOne = spline_comp->GetDistanceAlongSplineAtSplinePoint(FMath::TruncToInt(spline_comp->FindInputKeyClosestToWorldLocation(location) + 1.0f));

	return (DistAtTruncatedLoc + ( (DistAtTruncatedLocPlusOne - DistAtTruncatedLoc) * ( spline_comp->FindInputKeyClosestToWorldLocation(location) - FMath::TruncToInt(spline_comp->FindInputKeyClosestToWorldLocation(location)) ) ) );
}
float USplinePathInfoLib::getDirectionCheck(UAirSimSplineComp* spline_comp, FVector location, FRotator actor_rotation)
{
	FRotator SplineRot = spline_comp->FindTransformClosestToWorldLocation(location, ESplineCoordinateSpace::World).Rotator();
	return 1.0f * (FVector::DotProduct(SplineRot.Vector(), actor_rotation.Vector() ));
}
TArray<FViolationInfo> USplinePathInfoLib::checkForViolations(FSplinePathInfo spline_info, FVector actor_location, FRotator actor_rotation, float actor_speed)
{
	TArray<FViolationInfo> violations;

	if (spline_info.spline_comp_ != nullptr)
	{
		for (uint8 i = 0; i < (uint8)(EViolationType::VIOLATION_MAX); i++)
		{
			FViolationInfo new_violation = FViolationInfo((EViolationType)(i), 0.0f);
			//UE_LOG(LogTemp, Warning, TEXT("index %d"), i);
			switch (i)
			{
				case (uint8)EViolationType::VIOLATION_SPEED:
				{
					new_violation.current_violation_amount_ = FMath::Clamp(actor_speed - spline_info.spline_meta_data_.speed_limit, 0.0f, 99999999.0f);
					break;
				}
				case (uint8)EViolationType::VIOLATION_DIRECTION:
				{
					new_violation.current_violation_amount_ = 1.0f - getDirectionCheck(spline_info.spline_comp_, actor_location, actor_rotation);
					break;
				}
				case (uint8)EViolationType::VIOLATION_LANE:
				{
					new_violation.current_violation_amount_ = spline_info.dist_from_lane_;
					break;
				}
				case (uint8)EViolationType::VIOLATION_OFFROAD:
				{
					if (spline_info.all_spline_comps_.Num() == 1)
					{
						new_violation.current_violation_amount_ = spline_info.dist_from_lane_;
					}
					if (spline_info.all_spline_comps_.Num() == 0)
					{
						new_violation.current_violation_amount_ = 10000.0f;
					}
					break;
				}
				case (uint8)EViolationType::VIOLATION_REDLIGHT:
				{
					if (spline_info.next_traffic_light_status_ == ETrafficStatus::TRAFFIC_RED && spline_info.next_traffic_light_dist_ <= 0.0f)
					{
						new_violation.current_violation_amount_ = 1.0f;
					}
					break;
				}
			}
			violations.Add(new_violation);
		}
	
	}
	else
	{
		// we are totally offroad, no spline comp!
		FViolationInfo offroad_violation = FViolationInfo(EViolationType::VIOLATION_OFFROAD, 1000.0f);
		violations.Add(offroad_violation);
	}
	return violations;
}
float USplinePathInfoLib::velocityToMPH(FVector velocity)
{
	return velocity.Size() * 0.0223694f;
}
TArray<FViolationCount> USplinePathInfoLib::addToViolationCounts(TArray<FViolationCount> violation_counts, TArray<FViolationInfo> violations, bool in_intersection, float current_dist_on_spline)
{
	// find matching violation enum in ViolationCount array
	for (int32 i = 0; i < violations.Num(); i++)
	{
		for (int32 j = 0; j < violation_counts.Num(); j++)
		{
			if (violations[i].violation_type_ == violation_counts[j].violation_type_)
			{
				bool should_ignore_violation = false;
				if (violation_counts[j].ignore_in_intersection_ && in_intersection 
					|| violation_counts[j].min_dist_on_spline_ > current_dist_on_spline
					|| violation_counts[j].tolerance_ > violations[i].current_violation_amount_)
				{
					should_ignore_violation = true;
				}

				if (!should_ignore_violation)
				{
					if (violation_counts[j].current_violation_amount_ == 0.0f)
					{
						if (violations[i].current_violation_amount_ > 0.0f)
						{
							violation_counts[j].violation_count_++;
						}
					}
					violation_counts[j].current_violation_amount_ = violations[i].current_violation_amount_;
					
				}
				else
				{
					violation_counts[j].current_violation_amount_ = 0.0f;
				}
			}
		}
	}

	// compiler gets confused if we return violation_counts
	// note: unreal structs cannot be passed by reference so this is easier to understand anyways.
	TArray<FViolationCount> updated_violation_counts = violation_counts;
	return updated_violation_counts;
}
TArray<FViolationCount> USplinePathInfoLib::updateViolationCountsForVehicle(UWorld* world, AActor* vehicle, TArray<FViolationCount> current_violation_counts, float trace_size)
{
	float miles_per_hour = velocityToMPH(vehicle->GetVelocity());

	FVector parent_loc = vehicle->GetActorLocation();
	FRotator parent_rot = vehicle->GetActorRotation();

	FSplinePathInfo spline_path_info = getCurrentSplineInfo(world, vehicle->GetActorLocation(), vehicle->GetActorRotation(), trace_size);

	float dist_on_current_spline = getDistOnSplineClosestToLocation(vehicle->GetActorLocation(), spline_path_info.spline_comp_);

	TArray<FViolationInfo> current_violations = checkForViolations(spline_path_info, parent_loc, parent_rot, miles_per_hour);

	return addToViolationCounts(current_violation_counts, current_violations, spline_path_info.is_in_intersection, dist_on_current_spline);
}

TArray<FViolationCount> USplinePathInfoLib::getVehicleCurrentViolations(AActor* vehicle)
{
	ASplineViolationChecker* spline_violation_checker = getAttachedViolationChecker(vehicle);
	if (spline_violation_checker)
	{
		spline_violation_checker->violation_counts_;
	}
	
	TArray<FViolationCount> blank_violation_counts;
	
	return blank_violation_counts;

}
ASplineViolationChecker* USplinePathInfoLib::getAttachedViolationChecker(AActor* vehicle)
{
	ASplineViolationChecker* violation_checker = NULL;
	if (vehicle)
	{
		TArray<AActor*> attached_actors;
		vehicle->GetAttachedActors(attached_actors);

		for (int32 i = 0; i < attached_actors.Num(); i++)
		{
			ASplineViolationChecker* spline_violation_checker = Cast<ASplineViolationChecker>(attached_actors[i]);
			if (spline_violation_checker)
			{
				violation_checker = spline_violation_checker;
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("vehicle is null"));
	}

	if (!violation_checker)
	{
		UE_LOG(LogTemp, Warning, TEXT("getAttachedViolationChecker violation_checker is null"));
	}

	return violation_checker;
}
void USplinePathInfoLib::showViolationMenu(AActor* vehicle)
{
	ASplineViolationChecker* violation_checker = getAttachedViolationChecker(vehicle);
	if (violation_checker)
	{
		violation_checker->showMenu();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("violation_checker is null"));
	}
}

void USplinePathInfoLib::hideViolationMenu(AActor* vehicle)
{
	ASplineViolationChecker* violation_checker = getAttachedViolationChecker(vehicle);
	if (violation_checker)
	{
		violation_checker->hideMenu();
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("violation_checker is null"));
	}
}
void USplinePathInfoLib::initActorsCheckViolation(UWorld* world, TArray<AActor*> local_pawns)
{
	for (int32 i = 0; i < local_pawns.Num(); i++)
	{
		const FVector pawn_loc = local_pawns[i]->GetActorLocation();
		const FRotator pawn_rot = local_pawns[i]->GetActorRotation();

		FActorSpawnParameters spawn_param;
		spawn_param.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AActor* spawned_violation_checker = world->SpawnActor(ASplineViolationChecker::StaticClass(), &pawn_loc, &pawn_rot, spawn_param);

		ASplineViolationChecker* casted_violation_checker = Cast<ASplineViolationChecker>(spawned_violation_checker);

		if (casted_violation_checker)
		{
			casted_violation_checker->setVehicleToAttachTo(local_pawns[i]);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("casted_violation_checker is null"));
		}
	}
}