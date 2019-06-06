// Fill out your copyright notice in the Description page of Project Settings.

#include "BaseCarPath.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
//#include "BaseTrafficGroupControlVolume.h"
//#include "CityEnviron.h"


// Sets default values
ABaseCarPath::ABaseCarPath()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	CarPathSpline = CreateDefaultSubobject<UAirSimSplineComp>(TEXT("CarPathSpline"));

	RootComponent = CarPathSpline;

	SplineCollSphereSpacing = 150.0f;
	SplineCollSphereRadius = 100.0f;

	next_traffic_status_ = ETrafficStatus::TRAFFIC_GREEN;

	SetActorHiddenInGame(false);

	arrow_spacing_ = 250.0f;
	arrow_size_ = FVector(3.0f, 25.0f, 5.0f);
	arrow_color_ = FColor::Red;

	coll_mesh_scale_ = 2.0f;
	coll_mesh_spacing_scale_ = 1.0f;
	//coll_mesh_spacing_ = 50.0f;
	coll_mesh_tangent_length_ = 50.0f;
	coll_mesh_end_tangent_length_ = 75.0f;
	use_spline_coll_mesh_ = true;
}
void ABaseCarPath::OnConstruction(const FTransform & Transform)
{
	Super::OnConstruction(Transform);
	generateSplinesToConnectingPaths();
	// must run after generate splines, it requires splines or else it will crash!
	generateConnectingSplineCollSpheres();


	makeArrowCompsAlongSpline();

	makeCollMeshesAlongSpline();
}
UAirSimSplineComp* ABaseCarPath::getCarPathSpline()
{
	return CarPathSpline;
}
TArray<UAirSimSplineComp*> ABaseCarPath::GetEndConnectingSplines()
{
	TArray<UAirSimSplineComp*> end_connecting_splines;

	for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{
		end_connecting_splines.Add(end_connecting_paths_[i].connecting_spline);
	}

	return end_connecting_splines;
}
bool ABaseCarPath::spawnVehicle(FNPCVehicleType VehicleType, int32 VehicleIndex)
{
	// TODO: make a function for this - take into account vehicle size, speed
	float VehicleSpawnSafeDist = 700.0f + VehicleType.car_length_;

	float SpawnDist = CarPathSpline->GetSplineLength() - VehicleSpawnSafeDist;
	// get the last vehicle, and check if we can spawn behind it
	//GetLastVehicle();
	if (VehicleInstances.IsValidIndex(VehicleInstances.Num() - 1))
	{
		SpawnDist = VehicleInstances[VehicleInstances.Num() - 1].dist_along_spline_ - VehicleSpawnSafeDist;
	}

	if (SpawnDist > 0)
	{
		FTransform SpawnTransforms = CarPathSpline->GetTransformAtDistanceAlongSpline(SpawnDist, ESplineCoordinateSpace::World);

		if (MyVehicleManager->getAllAirsimCarPawns().Num() > 0)
		{
			TArray <AActor*> AirsimActors = MyVehicleManager->getAllAirsimCarPawns();
			for (int32 i = 0; i < AirsimActors.Num(); i++)
			{
				if (AirsimActors[i])
				{
					float DistToAirsimCar = FVector::Dist(AirsimActors[i]->GetActorLocation(), SpawnTransforms.GetLocation());
					// try to spawn at least this distance from the airsim car
					// TODO: Get airsim car's size!
					const float MinDistToAirsimCar = VehicleSpawnSafeDist;

					if (DistToAirsimCar < MinDistToAirsimCar)
					{
						return false;
					}
				}

			}
		}

		// make a copy for adding to the vehicle instances array
		FNPCVehicleInstance NewVehicleInstance;
		NewVehicleInstance.instanced_mesh_comp_ = VehicleType.hism_;
		// need world transform in here
		NewVehicleInstance.instance_index_ = NewVehicleInstance.instanced_mesh_comp_->AddInstanceWorldSpace(SpawnTransforms);

		NewVehicleInstance.dist_along_spline_ = SpawnDist;

		NewVehicleInstance.car_length_ = VehicleType.car_length_;

		NewVehicleInstance.max_speed_ = VehicleType.max_speed_;

		NewVehicleInstance.spot_light_offset_ = VehicleType.spot_light_offset_;

		if (MyVehicleManager)
		{
			// spawn a spotlight component
			USpotLightComponent* newSpotLight = MyVehicleManager->spawnCarLight();

			if (newSpotLight)
			{
				NewVehicleInstance.spot_light_ = newSpotLight;
			}
		}


		NewVehicleInstance.current_speed_variation_ = FMath::FRandRange(-1 * NewVehicleInstance.max_speed_variation_, NewVehicleInstance.max_speed_variation_);

		addVehicle(NewVehicleInstance);
		return true;
	}

	return false;


}
void ABaseCarPath::destroyVehicles()
{
	// remove spotlight, they don't get garbage collected
	for (int32 i = 0; i < VehicleInstances.Num(); i++)
	{
		if (VehicleInstances[i].spot_light_)
		{
			if (MyVehicleManager)
			{
				MyVehicleManager->removeCarLight(VehicleInstances[i].spot_light_);
			}
			VehicleInstances[i].spot_light_->DestroyComponent();
		}
	}

	VehicleInstances.Empty();
}
void ABaseCarPath::addVehicle(FNPCVehicleInstance VehicleInstance)
{
	if (VehicleInstance.instanced_mesh_comp_)
	{
		FNPCVehicleInstance NewVehicleInstance = VehicleInstance;
		NewVehicleInstance.is_on_connecting_spline_ = false;
		NewVehicleInstance.connecting_spline_index_ = -1;

		if (end_connecting_paths_.Num() > 0)
		{
			// clamp so it cannot go below 0, negative numbers cause invalid index later on and crashes!
			int32 rand_max = FMath::Clamp(end_connecting_paths_.Num() - 1, 0, 99999999);
			NewVehicleInstance.connecting_spline_index_ = FMath::RandRange(0, rand_max);
		}
		VehicleInstances.Add(NewVehicleInstance);
	}

}
bool ABaseCarPath::removeVehicle(FNPCVehicleInstance VehicleInstance)
{
	for (int32 i = 0; i < VehicleInstances.Num(); i++)
	{
		if (VehicleInstances[i].Compare(VehicleInstance))
		{
			// do NOT remove spotlight, remove vehicles just removes me from this path
			VehicleInstances.RemoveAt(i);
			return true;
		}
	}
	return false;
}
FNPCVehicleInstance ABaseCarPath::GetLastVehicle()
{
	if (VehicleInstances.Num() == 0)
	{
		FNPCVehicleInstance NullVehicleInstance;
		return NullVehicleInstance;
	}
	if (VehicleInstances.Num() == 1)
	{
		return VehicleInstances[0];
	}
	return VehicleInstances[VehicleInstances.Num() - 1];
}
bool ABaseCarPath::checkConnectingPathCollision(int32 VehicleIndex, bool EnableCollisionsIfSuccessful)
{
	// TODO: add those new checks involving the new comp, and use that comp's enable and disable functions

	// TODO: ONLY check the spheres in front of the car

	// TODO: UPDATE ALL CONNECTING PATH CARS: get all cars on the connecting path, check if predicted collision, if so stop

	const int32 PathIndex = VehicleInstances[VehicleIndex].connecting_spline_index_;

	int32 StartingSphereIndex = 0;

	if (VehicleInstances[VehicleIndex].is_on_connecting_spline_)
	{
		//TODO: need to set this to a real value based on the real vehicle size
		// subtract from starting position so the sphere checks a little bit behind me as well (since origin is at centre of vehicle)
		float VehicleLength = VehicleInstances[VehicleIndex].car_length_;
		StartingSphereIndex = (VehicleInstances[VehicleIndex].dist_along_spline_ - (VehicleLength * 0.5)) / SplineCollSphereSpacing;

		// dont allow 0 or less, will get invalid index
		StartingSphereIndex = FMath::Clamp(StartingSphereIndex, 0, 9999);
	}

	// arbitrary number, replace me with real speed later please
	//hmmm... seems to cause issues when we make it too low. perhaps we need a clamp to make sure it still checks a few in front?
	// need to make sure it still checks a little bit ahead even if our speed is 0
	const float Speed = 900.0f;// VehicleInstances[VehicleIndex].CurrentSpeed;

	// arbitrary how many seconds ahead to check - replace with real number please
	float SecondsToCheck = 1.5f;

	int32 LastSphereIndex = FMath::Clamp((StartingSphereIndex + (SecondsToCheck * Speed + VehicleInstances[VehicleIndex].dist_along_spline_) / SplineCollSphereSpacing),
		(float)StartingSphereIndex, (float) (end_connecting_paths_[PathIndex].CollSpheres.Num()) - 1);

	// TODO: check for car collisions, but dont worry about cars behind us on this path (cars in front of us matter though)

	bool bFoundOverlap = false;

	for (int32 i = StartingSphereIndex; i < LastSphereIndex; i++)
	{
		// if the collision was already disabled, set the collision to disabled again after checking
		bool ShouldDisableCollisionAfterCheck = false;

		if (!end_connecting_paths_[PathIndex].CollSpheres[i]->GetCollisionEnabled())
		{
			ShouldDisableCollisionAfterCheck = true;
		}

		// enable collision, test overlaps, then disable it again
		end_connecting_paths_[PathIndex].CollSpheres[i]->enableCollision(0.0f);
		TArray<AActor*> OverlappingActors;
		end_connecting_paths_[PathIndex].CollSpheres[i]->GetOverlappingActors(OverlappingActors);


		TArray<UPrimitiveComponent*> OverlappingComponents;
		end_connecting_paths_[PathIndex].CollSpheres[i]->GetOverlappingComponents(OverlappingComponents);



		bFoundOverlap = (OverlappingActors.Num() != 0 && !(end_connecting_paths_[PathIndex].CollSpheres[i]->IsOverlappingActor(this) && OverlappingActors.Num() == 1));

		if (ShouldDisableCollisionAfterCheck)
		{
			end_connecting_paths_[PathIndex].CollSpheres[i]->disableCollision();
		}
		if (bFoundOverlap)
		{
			break;
		}
	}

	if (bFoundOverlap)
	{
		return true;
	}
	else if (EnableCollisionsIfSuccessful)
	{
		for (int32 i = StartingSphereIndex; i < LastSphereIndex; i++)
		{
			// totally arbitrary number
			// TODO: intelligently estimate how long it will take before we go over this?
			const float Duration = 1.5f;
			end_connecting_paths_[PathIndex].CollSpheres[i]->enableCollision(Duration);
		}
	}
	return false;
}
/*void ABaseCarPath::DisableAllSphereColl()
{
	//EndConnectingPaths[PathIndex].CollSpheres[i];
	if (EndConnectingPaths.Num() == 0)
	{
		return;
	}
	for (int32 i = 0; i < EndConnectingPaths.Num() - 1; i++)
	{
		//EndConnectingPaths[i];
		for (int32 j = 0; j < EndConnectingPaths[i].CollSpheres.Num() - 1; j++)
		{
			EndConnectingPaths[i].CollSpheres[j]->DisableCollision();
		}
	}
}*/
void ABaseCarPath::setIntersection(ABaseIntersection* NewIntersection)
{
	NextIntersection = NewIntersection;
}
ABaseIntersection* ABaseCarPath::getIntersection()
{
	return NextIntersection;
}
bool ABaseCarPath::nextWaitingVehicleStartMovingOnConnectingSpline()
{
	for (int32 i = 0; i < VehicleInstances.Num(); i++)
	{
		// if we are NOT bIsOnConnectingSpline but at the end of the spline, it means we are waiting at an intersection to go.
		if (!VehicleInstances[i].is_on_connecting_spline_ && VehicleInstances[i].dist_along_spline_ >= CarPathSpline->GetSplineLength())
		{
			if (end_connecting_paths_.Num() == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("%s HAS NO CONNECTING PATHS!"), *GetFName().ToString());
				return false;
			}

			// if we havent set a connecting spline index yet, pick a random one now
			// changed to pick a random one when added to a path
			/*if (VehicleInstances[i].ConnectingSplineIndex == -1)
			{
				// MUST clamp! prevent it from going below 0 if we have 0 or no connecting paths
				VehicleInstances[i].ConnectingSplineIndex = FMath::Clamp((FMath::RandRange(0, EndConnectingPaths.Num() - 1)), 0, 9999999);
				UE_LOG(LogTemp, Warning, TEXT("%d generating new connecting spline index!"), VehicleInstances[i].ConnectingSplineIndex);
			}*/
			if (!checkConnectingPathCollision(i))
			{
				// start moving on the connecting spline!!
				VehicleInstances[i].is_on_connecting_spline_ = true;
				// remember to reset dist along spline or we'll teleport suddenly
				VehicleInstances[i].dist_along_spline_ = 0;
				return true;
			}

			// TODO: keep checking this while updating vehicle instances on the path!!!

			/*if (!checkConnectingPathCollision(VehicleInstances[i].ConnectingSplineIndex))
			{
				// start moving on the connecting spline!!
				VehicleInstances[i].bIsOnConnectingSpline = true;
				// remember to reset dist along spline or we'll teleport suddenly
				VehicleInstances[i].DistAlongSpline = 0;
				return true;
			}*/
		}
	}
	return false;
}
// Called when the game starts or when spawned
void ABaseCarPath::BeginPlay()
{
	Super::BeginPlay();

	if (!getIntersection())
	{
		UE_LOG(LogTemp, Warning, TEXT("INVALID intersection! Remember to init intersections, or violations won't get next traffic light correctly!"));
	}
}
void ABaseCarPath::generateConnectingSplineCollSpheres()
{
	// sometimes destroy component doesn't happen right away - having it in a separate loop resolves that
	for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{
		for (int32 j = 0; j < end_connecting_paths_[i].CollSpheres.Num(); j++)
		{
			if (end_connecting_paths_[i].CollSpheres.IsValidIndex(j) && end_connecting_paths_[i].CollSpheres[j]->IsValidLowLevel())
			{
				end_connecting_paths_[i].CollSpheres[j]->DestroyComponent();
				if (end_connecting_paths_[i].CollSpheres[j])
				{
					end_connecting_paths_[i].CollSpheres[j]->SetActive(false);
				}
			}
		}
	}

	for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{

		end_connecting_paths_[i].CollSpheres.Empty();

		int32 MaxNumCollSpheres = 30;
		int32 NumCollSpheres = FMath::Clamp(static_cast<int32>(GetEndConnectingSplines()[i]->GetSplineLength() / SplineCollSphereSpacing), 0, MaxNumCollSpheres);

		for (int32 j = 0; j < NumCollSpheres; j++)
		{
			class UBaseCarPathCollSphereComponent* NewSphereComp = NewObject<UBaseCarPathCollSphereComponent>(this);
			if (NewSphereComp)
			{
				// prevents it from being garbage collected
				NewSphereComp->RegisterComponent();
				//NewSphereComp->AttachTo(RootComponent);

				FTransform NewSphereTransform = GetEndConnectingSplines()[i]->GetTransformAtDistanceAlongSpline(j * SplineCollSphereSpacing, ESplineCoordinateSpace::World);
				NewSphereComp->AttachToComponent(RootComp, FAttachmentTransformRules::KeepRelativeTransform);
				NewSphereComp->SetSphereRadius(SplineCollSphereRadius, false);
				NewSphereComp->SetWorldLocation(NewSphereTransform.GetLocation());
				// by default no collisions. we only enable them when we want to test for collision
				// TODO: blueprint to override this in case we have different collision profiles
				NewSphereComp->SetCollisionProfileName("CarPathCollSphere");
				// make sure to do this last, setting collision profile will override this!!
				NewSphereComp->disableCollision();
				// this should make it ignore our own stuff
				// nvm this is just for movement, doesnt really work...
				//NewSphereComp->IgnoreActorWhenMoving(this, true);

				NewSphereComp->bGenerateOverlapEvents = true;
				//NewSphereComp->Overlap
			}


			end_connecting_paths_[i].CollSpheres.Add(NewSphereComp);
		}
		
	}
}
void ABaseCarPath::cleanupOldSplines()
{
	TArray<UActorComponent*> spline_comps = GetComponentsByClass(UAirSimSplineComp::StaticClass());

	// get rid of all temporary splines that arent the main spline
	for (int32 i = 0; i < spline_comps.Num(); i++)
	{
		// TODO: check if this works on package!
		if (spline_comps[i]->GetFName() != TEXT("CarPathSpline"))
		{
			spline_comps[i]->SetActive(false);
			spline_comps[i]->DestroyComponent();
			UE_LOG(LogTemp, Warning, TEXT("destroying old spline"));
		}
	}

	TArray<UActorComponent*> sphere_comps = GetComponentsByClass(UBaseCarPathCollSphereComponent::StaticClass());

	// get rid of all temporary splines that arent the main spline
	for (int32 i = 0; i < sphere_comps.Num(); i++)
	{
		// TODO: check if this works on package!
		if (sphere_comps[i]->GetFName() != TEXT("CarPathSpline"))
		{
			sphere_comps[i]->SetActive(false);
			sphere_comps[i]->DestroyComponent();
			UE_LOG(LogTemp, Warning, TEXT("destroying old sphere comps"));
		}
	}

}
void ABaseCarPath::generateSplinesToConnectingPaths()
{
	if (!manually_edit_spline_connector_points_)
	{
		spline_transform_widgets_.Empty();
	}

	spline_transform_widget_index = 0;

	TArray<UActorComponent*> comps;
	GetComponentsByClass(UAirSimSplineComp::StaticClass());

	// sometimes destroy component doesn't happen right away - having it in a separate loop resolves that
	for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{
		if (end_connecting_paths_[i].connecting_spline->IsValidLowLevel())
		{
			end_connecting_paths_[i].connecting_spline->DestroyComponent();
			if (end_connecting_paths_[i].connecting_spline)
			{
				end_connecting_paths_[i].connecting_spline->SetActive(false);
			}

			end_connecting_paths_[i].connecting_spline = NULL;
			UE_LOG(LogTemp, Warning, TEXT("destroying old spline"));
		}
	}

	for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{
		//add airsim spline comp
		UAirSimSplineComp* new_spline_comp = NewObject<UAirSimSplineComp>(this);
		if (new_spline_comp)
		{
			// prevents garbage collection during runtime
			new_spline_comp->RegisterComponent();
			new_spline_comp->AttachTo(RootComponent);
			new_spline_comp->bDrawDebug = true;

			UE_LOG(LogTemp, Warning, TEXT("spawning new spline"));
			new_spline_comp->ClearSplinePoints(true);

			// replaces the blueprint array for OLDEndConnectingSplines
			end_connecting_paths_[i].connecting_spline = new_spline_comp;

			// end of CAR PATH spline location (this path, NOT the connecting path), which should be the start location of the connecting path.
			FVector start_location = CarPathSpline->GetLocationAtDistanceAlongSpline(CarPathSpline->GetSplineLength(), ESplineCoordinateSpace::World);

			// NOTE: keep this order, of first mid and end otherwise they spline points will be jumbled!
			// add one spline point to end
			new_spline_comp->AddSplinePoint(start_location, ESplineCoordinateSpace::World, true);

			if (end_connecting_paths_[i].CarPath && end_connecting_paths_[i].CarPath->getCarPathSpline())
			{
				// final spline point
				// don't add until the mid point is added!
				FVector end_location = end_connecting_paths_[i].CarPath->getCarPathSpline()->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);

				//UE_LOG(LogTemp, Warning, TEXT("%s final spline pos location"), *end_location.ToString());

				//mid point(s)
				int32 num_midpoints = end_connecting_paths_[i].NumSplineMidPoints;
				if (manually_edit_spline_connector_points_ && spline_transform_widgets_.IsValidIndex(spline_transform_widget_index))
				{
					// starts at 1, since 0 is already used for first point
					for (int32 j = 1; j <= num_midpoints; j++)
					{
						//new_spline_comp->AddSplinePoint(spline_transform_widgets_[spline_transform_widget_index].GetLocation(), ESplineCoordinateSpace::World, true);

						new_spline_comp->AddSplinePoint(spline_transform_widgets_[spline_transform_widget_index].GetLocation(), ESplineCoordinateSpace::Local, true);
						// for some reason when we set coord space to local, the spline shows up. when we set it to world, the spline disappears??/

						/*UE_LOG(LogTemp, Warning, TEXT("set spline point to transform point"));
						UE_LOG(LogTemp, Warning, TEXT("%d num points"), new_spline_comp->GetNumberOfSplinePoints());
						UE_LOG(LogTemp, Warning, TEXT("%d current transform widget index"), spline_transform_widget_index);
						UE_LOG(LogTemp, Warning, TEXT("%s trasform location"), *spline_transform_widgets_[spline_transform_widget_index].GetLocation().ToString());
						UE_LOG(LogTemp, Warning, TEXT("%s spline point 1 location"), *new_spline_comp->GetLocationAtSplinePoint(1, ESplineCoordinateSpace::World).ToString());*/


						// for some reason we get a spline point that shows up right at 0,0,0 no matter what we do here...

						spline_transform_widget_index++;

					}
				}
				// if false, just put a bunch of points between the start and end
				else
				{
					// starts at 1, since 0 is already used for first point
					for (int32 j = 1; j <= num_midpoints; j++)
					{
						FVector new_location = (((end_location - start_location) / (num_midpoints + 1)) * j) + start_location;

						new_spline_comp->AddSplinePoint(new_location, ESplineCoordinateSpace::World, true);

						new_location = GetActorTransform().InverseTransformPosition(new_location);

						// have to make a transform for it to be added to widget
						FTransform new_transform = FTransform(FRotator(0, 0, 0), new_location, FVector(1, 1, 1));
						// need to inverse transform locatio nfirst
						spline_transform_widgets_.Add(new_transform);

						UE_LOG(LogTemp, Warning, TEXT("added transform"));
					}
				}
				// finally add the end point
				new_spline_comp->AddSplinePoint(end_location, ESplineCoordinateSpace::World, true);
				/*UE_LOG(LogTemp, Warning, TEXT("adding final spline pos"));
				UE_LOG(LogTemp, Warning, TEXT("%s final spline pos location"), *end_location.ToString());*/


			}
			else
			{
				// gets to here,
				UE_LOG(LogTemp, Warning, TEXT("no car path"));

			}
		}
	}
}
void ABaseCarPath::regenConnectingSplines()
{
	cleanupOldSplines();
	manually_edit_spline_connector_points_ = false;
	generateSplinesToConnectingPaths();
	manually_edit_spline_connector_points_ = true;
	generateSplinesToConnectingPaths();
	generateConnectingSplineCollSpheres();

	/*for (int32 i = 0; i < end_connecting_paths_.Num(); i++)
	{
		if (spline_transform_widgets_.IsValidIndex(i))
		{
			
			FVector last_spline_loc = CarPathSpline->GetLocationAtDistanceAlongSpline(CarPathSpline->GetSplineLength(), ESplineCoordinateSpace::World);
			FRotator last_spline_rot = CarPathSpline->GetRotationAtDistanceAlongSpline(CarPathSpline->GetSplineLength(), ESplineCoordinateSpace::World);
			FVector connecting_spline_start_loc = end_connecting_paths_[i].connecting_spline->GetLocationAtDistanceAlongSpline(end_connecting_paths_[i].connecting_spline->GetSplineLength(), ESplineCoordinateSpace::World);

			// hypotenuse
			float distance = (last_spline_loc - connecting_spline_start_loc).Size();

			// cos angle_between_vectors = adj / hyp

			//FVector::DotProduct(last_spline_loc.GetSafeNormal(), connecting_spline_start_loc);
			float angle_between_vectors = UKismetMathLibrary::DegAcos(FVector::DotProduct(last_spline_loc.GetSafeNormal(), connecting_spline_start_loc.GetSafeNormal()));

			float adjacent_length = (FMath::Cos(angle_between_vectors)) * distance;
			FVector forward_vector = UKismetMathLibrary::GetForwardVector(last_spline_rot);

			//UKismetMathLibrary::InverseTransformLocation(GetActorTransform(), forward_vector * adjacent_length + last_spline_loc);
			spline_transform_widgets_[i].SetLocation(UKismetMathLibrary::InverseTransformLocation(GetActorTransform(), (forward_vector * adjacent_length) + last_spline_loc));
			//end_connecting_paths_[i].
			//spline_transform_widgets_[i].SetLocation();
		}
	}*/
}
int32 ABaseCarPath::getLaneNumber() const
{
	return LaneNumber;
}

void ABaseCarPath::updateVehicleInstances(float DeltaSeconds)
{
	// indexes of vehicles that are leaving this spline for next spline
	TArray<FNPCVehicleInstance> VehiclesNextSpline;


	for (int32 i = 0; i < VehicleInstances.Num(); i++)
	{
		bool bShouldStop = false;
		bool bForceStop = false;
		UAirSimSplineComp* CurrentSplineToMoveOn = CarPathSpline;

		// connecting spline is set, set it to move on that one instead
		if (VehicleInstances[i].connecting_spline_index_ != -1 && VehicleInstances[i].is_on_connecting_spline_)
		{
			// TODO: slow down instead of suddenly slamming our brakes
			// TODO: kind of messy, we force collisions enabled here... but it doesn't necessarily mean we are going fwd
			if (checkConnectingPathCollision(i, true))
			{
				bShouldStop = true;
			}
			if (GetEndConnectingSplines().IsValidIndex(VehicleInstances[i].connecting_spline_index_))
			{
				CurrentSplineToMoveOn = GetEndConnectingSplines()[VehicleInstances[i].connecting_spline_index_];
				// if we reached the end of this spline we need to put this vehicle onto the next spline
				if (VehicleInstances[i].dist_along_spline_ >= CurrentSplineToMoveOn->GetSplineLength())
				{
					VehiclesNextSpline.Add(VehicleInstances[i]);
				}
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("%s HAS INVALID CONNECTING SPLINE!"), *GetFName().ToString());
			}
		}
		// 4/23/2019 - check ahead by a bit for collisions, and stop to avoid collision
		else if (!VehicleInstances[i].is_on_connecting_spline_ && VehicleInstances[i].dist_along_spline_ + VehicleInstances[i].current_speed_ >= CurrentSplineToMoveOn->GetSplineLength())
		{
			if (checkConnectingPathCollision(i, false))
			{
				bShouldStop = true;
			}
		}

		// get distance to all airsim vehicles
		TArray<AActor*> AirsimActors = MyVehicleManager->getAllAirsimCarPawns();

		if (AirsimActors.Num() > 0)
		{
			for (int32 j = 0; j < AirsimActors.Num(); j++)
			{
				if (AirsimActors[j])
				{
					FTransform InstanceTransform;
					VehicleInstances[i].instanced_mesh_comp_->GetInstanceTransform(VehicleInstances[i].instance_index_, InstanceTransform, true);
					float Distance = FVector::Dist(AirsimActors[j]->GetActorLocation(), InstanceTransform.GetLocation());

					// Neighbourhood hack: increase collision check dist, so we can collide with animals
					float CheckCollDist = 15000.0f; //1800.0f + VehicleInstances[i].car_length_;

					float MinCheckCollDist = 700.0f + VehicleInstances[i].car_length_;

					const float TraceDist = FMath::Clamp(VehicleInstances[i].current_speed_ * 2.0f, MinCheckCollDist, CheckCollDist);

					//bool bTraceHitAirsimCar = false;
					if (Distance <= CheckCollDist && GetWorld())
					{
						FHitResult HitResult;

						// trace up a bit, because the paths are right on the ground
						float zBoost = 50.0f;

						FCollisionQueryParams CollisionParams = FCollisionQueryParams::DefaultQueryParam;
						CollisionParams.AddIgnoredActor(MyVehicleManager);
						CollisionParams.TraceTag = TEXT("AirsimPawnTrace");

						int32 NumTraces = 5;

						const float TraceMaxY = 120.0f;
						for (int k = 0; k < NumTraces; k++)
						{
							FTransform TraceStartTransform = InstanceTransform;

							FVector DeltaTranslation = FVector(0, (0 - ((TraceMaxY) / 2) + (k * (TraceMaxY / NumTraces))), 0);
							TraceStartTransform.AddToTranslation(DeltaTranslation);
							//FVector TraceStartVector = TraceStartTransform.InverseTransformVector(FVector(0, 0 - (TraceMaxY/2 * NumTraces) + (k * TraceMaxY/NumTraces), 0));
							FVector EndTrace = (TraceStartTransform.GetRotation().GetForwardVector() * TraceDist) + TraceStartTransform.GetLocation() + FVector(0, 0, zBoost);


							bool TraceHit = GetWorld()->LineTraceSingleByChannel(HitResult, TraceStartTransform.GetLocation() + FVector(0, 0, zBoost), EndTrace, ECC_Visibility, CollisionParams);
							// draw the line trace
							//GetWorld()->DebugDrawTraceTag = CollisionParams.TraceTag;
							if (TraceHit && HitResult.GetActor())
							{
								APawn* TestPawn = Cast<APawn>(HitResult.GetActor());
								if (TestPawn)
								{
									//UE_LOG(LogTemp, Warning, TEXT("%s SHOULD STOP! HAS AIRSIM CAR IN FRONT"), *GetFName().ToString());
									bShouldStop = true;
									break;
								}
							}
						}
					}
				}
			}
		}



		// TODO: make a function for this - take into account vehicle size, speed
		// try to stop 1 seconds before hitting the next vehicle
		// currently, dist to next vehicle already takes into account this vehicle's length and the next vehicles length
		float NextVehicleMinDist = 700.0f;

		float NextVehicleMaxDist = NextVehicleMinDist;
		if (VehicleInstances[i].max_speed_ > NextVehicleMaxDist)
		{
			NextVehicleMaxDist = VehicleInstances[i].max_speed_;
		}

		float StopBeforeNextVehicleDist = FMath::Clamp(VehicleInstances[i].current_speed_ * 2.0f, NextVehicleMinDist, NextVehicleMaxDist);
		// if we have a non green light and we're not on a connecting spline, AND we are at the END of a path (at intersection basically)
		if (next_traffic_status_ != ETrafficStatus::TRAFFIC_GREEN && !VehicleInstances[i].is_on_connecting_spline_ && VehicleInstances[i].dist_along_spline_ >= CurrentSplineToMoveOn->GetSplineLength())
		{
			bShouldStop = true;
		}
		// negative values are invalid, so ignore those (it means theres no next car)
		if (GetDistanceToNextVehicle(i) <= StopBeforeNextVehicleDist && GetDistanceToNextVehicle(i) > 0.0f)
		{
			//UE_LOG(LogTemp, Warning, TEXT("%s SHOULD STOP! HAS VEHICLE IN FRONT"), *GetFName().ToString());
			bShouldStop = true;
			//slam brakes if the next car is very very close
			if (GetDistanceToNextVehicle(i) <= 100.0f)
			{
				bForceStop = true;
			}
		}



		// ------------------------------
		// VEHICLE MOVEMENT CODE BEGINS HERE
		// ------------------------------

		// finally, moving on spline
		int32 instnace_index_ = VehicleInstances[i].instance_index_;

		float current_speed_ = FMath::Clamp( (VehicleInstances[i].current_speed_ - (DeltaSeconds * VehicleInstances[i].getDecelerationRate()) ), 0.0f, VehicleInstances[i].max_speed_);



		if (!bShouldStop)
		{
			current_speed_ = FMath::Clamp((VehicleInstances[i].current_speed_ + (DeltaSeconds * VehicleInstances[i].acceleration_rate_ * (1.0f + VehicleInstances[i].current_speed_variation_))), 0.0f, VehicleInstances[i].max_speed_);
		}
		if (bForceStop)
		{
			current_speed_ = 0.0f;
		}
		if (current_speed_ > 0.0f)
		{
			float NewDistance = VehicleInstances[i].dist_along_spline_ + DeltaSeconds * current_speed_;
			VehicleInstances[i].dist_along_spline_ = NewDistance;
			FTransform NewTransform = CurrentSplineToMoveOn->GetTransformAtDistanceAlongSpline(NewDistance, ESplineCoordinateSpace::World);
			FTransform LastTransform;
			VehicleInstances[i].instanced_mesh_comp_->GetInstanceTransform(VehicleInstances[i].instance_index_, LastTransform, true);

			float TurnRate = 3.0f * DeltaSeconds;
			FRotator NewRotator = UKismetMathLibrary::RLerp(LastTransform.Rotator(), NewTransform.Rotator(), TurnRate, true);
			//NewTransform.Rotator = NewRotator;
			NewTransform = FTransform(NewRotator, NewTransform.GetTranslation(), NewTransform.GetScale3D());

			if (VehicleInstances[i].spot_light_)
			{
				//NewTransform.GetLocation().ForwardVector
				//NewTransform.
				//VehicleInstances[i].spot_light_->SetWorldLocation(NewTransform.GetTranslation());// +VehicleInstances[i].spot_light_offset_);
				VehicleInstances[i].spot_light_->SetWorldRotation(NewRotator);
				VehicleInstances[i].spot_light_->SetWorldLocation(UKismetMathLibrary::TransformLocation(NewTransform, VehicleInstances[i].spot_light_offset_));
			}

			VehicleInstances[i].instanced_mesh_comp_->UpdateInstanceTransform(instnace_index_, NewTransform, false);
		}


		// set the vehicles current speed last after we've moved it and done all that math above
		VehicleInstances[i].current_speed_ = current_speed_;
		// ------------------------------
		// END OF VEHICLE MOVEMENT CODE
		// ------------------------------

	}


	// remove all from this path
	for (int32 i = 0; i < VehiclesNextSpline.Num(); i++)
	{
		if (removeVehicle(VehiclesNextSpline[i]))
		{
			//UE_LOG(LogTemp, Warning, TEXT("REMOVE SUCCESSFUL!"));
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("REMOVE FAILED!"));
		}
	}
	// add to next path
	for (int32 i = 0; i < VehiclesNextSpline.Num(); i++)
	{
		if (VehiclesNextSpline[i].instanced_mesh_comp_ && end_connecting_paths_[VehiclesNextSpline[i].connecting_spline_index_].CarPath)
		{
			// reset the distance so we dont teleport halfway onto the next one
			VehiclesNextSpline[i].dist_along_spline_ = 0;
			end_connecting_paths_[VehiclesNextSpline[i].connecting_spline_index_].CarPath->addVehicle(VehiclesNextSpline[i]);
			//UE_LOG(LogTemp, Warning, TEXT("ADDING TO NEXT PATH"));
		}

	}

}
float ABaseCarPath::GetDistanceToNextVehicle(int32 Index) 
{
	FTransform VehicleTransform;
	VehicleInstances[Index].instanced_mesh_comp_->GetInstanceTransform(VehicleInstances[Index].instance_index_, VehicleTransform, true);
	FTransform NextVehicleTransform;


	// if the index is 0, it means we must be the lead vehicle, so no vehicle in front of us
	if (Index == 0)
	{
		if (!end_connecting_paths_.IsValidIndex(VehicleInstances[Index].connecting_spline_index_) || !end_connecting_paths_[VehicleInstances[Index].connecting_spline_index_].CarPath)
		{
			return -1;
		}
		//return -1;
		FNPCVehicleInstance NextPathLastVehicle = end_connecting_paths_[VehicleInstances[Index].connecting_spline_index_].CarPath->GetLastVehicle();
		// no real way to check if we got a valid nextpath vehicle or not... just check if the instance index is invalid
		// replaced with bIsOnConnectingSpline, since the index doesnt really tell if we are on the connecting spline or not, just that we want to go on it
		if (VehicleInstances[Index].is_on_connecting_spline_ //VehicleInstances[Index].ConnectingSplineIndex != -1
			&& NextPathLastVehicle.instance_index_ != -1)
		{
			//UE_LOG(LogTemp, Warning, TEXT("%d NEXT INSTANCE INDEX!"), NextPathLastVehicle.InstanceIndex );
			if (NextPathLastVehicle.instanced_mesh_comp_->GetInstanceTransform(NextPathLastVehicle.instance_index_, NextVehicleTransform, true) )
			{
				NextVehicleTransform.GetLocation();
				// TODO: check spline distance, not just distance check (this assumes its a straight line from eacH)
				//return FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation());
				float Dist = FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation()) - (NextPathLastVehicle.car_length_/2 + VehicleInstances[Index].car_length_/2);
				return FMath::Clamp(Dist, 0.0f, 999999.0f);
			}
		}
		//VehicleInstances[Index].ConnectingSplineIndex
		return -1;
	}
	if (VehicleInstances.IsValidIndex(Index - 1))
	{
		VehicleInstances[Index - 1].instanced_mesh_comp_->GetInstanceTransform(VehicleInstances[Index - 1].instance_index_, NextVehicleTransform, true);
		//UE_LOG(LogTemp, Warning, TEXT("%d NEXT INSTANCE INDEX!"), NextPathLastVehicle.InstanceIndex);
		//UE_LOG(LogTemp, Warning, TEXT("dist equals %f"), FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation() ) );
		//return FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation());
		float Dist = FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation()) - (VehicleInstances[Index - 1].car_length_/2 + VehicleInstances[Index].car_length_/2);
		return FMath::Clamp(Dist, 0.0f, 999999.0f);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("%s got an invalid index checking dist to next vehicle!"), *GetFName().ToString());
		return -1;
	}
	return -1;
}
float ABaseCarPath::GetNextVehicleLength(int32 Index)
{
	if (Index == 0)
	{
		if (!end_connecting_paths_.IsValidIndex(VehicleInstances[Index].connecting_spline_index_) || !end_connecting_paths_[VehicleInstances[Index].connecting_spline_index_].CarPath)
		{
			return -1;
		}
		FNPCVehicleInstance NextPathLastVehicle = end_connecting_paths_[VehicleInstances[Index].connecting_spline_index_].CarPath->GetLastVehicle();
		// no real way to check if we got a valid nextpath vehicle or not... just check if the instance index is invalid
		// replaced with bIsOnConnectingSpline, since the index doesnt really tell if we are on the connecting spline or not, just that we want to go on it
		if (VehicleInstances[Index].is_on_connecting_spline_ //VehicleInstances[Index].ConnectingSplineIndex != -1
			&& NextPathLastVehicle.instance_index_ != -1)
		{
			return NextPathLastVehicle.car_length_;
		}
		return -1;
	}
	if (VehicleInstances.IsValidIndex(Index - 1))
	{
		//VehicleInstances[Index - 1].InstancedMeshComp->GetInstanceTransform(VehicleInstances[Index - 1].InstanceIndex, NextVehicleTransform, true);
		//return FVector::Dist(VehicleTransform.GetLocation(), NextVehicleTransform.GetLocation());
		return VehicleInstances[Index - 1].car_length_;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("%s got an invalid index checking next vehicle!"), *GetFName().ToString());
		return -1;
	}

	return 0.0f;
}
// Called every frame
void ABaseCarPath::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	updateVehicleInstances(DeltaTime);

	// we dont have an intersection, so we should just move on our own
	if (!NextIntersection && GetTrafficLightStatus() == ETrafficStatus::TRAFFIC_GREEN)
	{
		nextWaitingVehicleStartMovingOnConnectingSpline();
	}
}
void ABaseCarPath::makeArrowCompsAlongSpline()
{
	// makes sure that they get totally destroyed and garbage collected.
	for (int32 i = 0; i < arrow_components_.Num(); i++)
	{
		if (arrow_components_[i])
		{
			arrow_components_[i]->DestroyComponent();

			// sometimes gets immediately destroyed, sometimes doesn't
			if (arrow_components_[i])
			{
				arrow_components_[i]->SetActive(false);
				arrow_components_[i]->UnregisterComponent();
			}
		}
	}


	arrow_components_.Empty();

	float arrow_dist_on_spline = 0;

	int32 num_arrow_comps = CarPathSpline->GetSplineLength() / arrow_spacing_;

	for (int32 i = 0; i <= num_arrow_comps; i++)
	{
		//add airsim spline comp
		UArrowComponent* new_arrow_comp = NewObject<UArrowComponent>(this);
		if (new_arrow_comp)
		{
			// prevents garbage collection during runtime
			new_arrow_comp->RegisterComponent();
			new_arrow_comp->AttachTo(RootComponent);

			new_arrow_comp->SetRelativeLocation(CarPathSpline->GetLocationAtDistanceAlongSpline(arrow_dist_on_spline, ESplineCoordinateSpace::Local));
			new_arrow_comp->SetRelativeRotation(CarPathSpline->GetRotationAtDistanceAlongSpline(arrow_dist_on_spline, ESplineCoordinateSpace::Local));
			new_arrow_comp->SetRelativeScale3D(arrow_size_);
			//CarPathSpline->GetLocationAtDistanceAlongSpline(arrow_dist_on_spline, ESplineCoordinateSpace::Local);
			//CarPathSpline->GetRotationAtDistanceAlongSpline(arrow_dist_on_spline, ESplineCoordinateSpace::Local);

			//new_arrow_comp->ArrowColor = arrow_color_;
			new_arrow_comp->SetArrowColor(arrow_color_);

			arrow_dist_on_spline = i * arrow_spacing_;

			arrow_components_.Add(new_arrow_comp);
		}
	}

}
void ABaseCarPath::generateSplineCompMetaData()
{
	CarPathSpline->generateMetaData();
}
void ABaseCarPath::generateAllSplineCompMetaData()
{
	if (GetWorld())
	{
		TArray<AActor*> CarPaths;

		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABaseCarPath::StaticClass(), CarPaths);

		for (int32 i = 0; i < CarPaths.Num(); i++)
		{
			ABaseCarPath* CastedCarPath = Cast<ABaseCarPath>(CarPaths[0]);
			if (CastedCarPath && CastedCarPath->getCarPathSpline())
			{
				CastedCarPath->getCarPathSpline()->generateMetaData();
			}
		}

	}

}
void ABaseCarPath::applySpeedLimitToSelectedPoint()
{
	int32 last_index = CarPathSpline->last_selected_index;

	FAirSimSplinePointMetaData spline_metadata = CarPathSpline->spline_metadata_[last_index];

	spline_metadata.speed_limit = new_speed_limit_;

	CarPathSpline->spline_metadata_[last_index] = spline_metadata;
}

void ABaseCarPath::applySpeedLimitToAll()
{
	for (int32 i = 0; i < CarPathSpline->GetNumberOfSplinePoints(); i++)
	{
		FAirSimSplinePointMetaData spline_metadata = CarPathSpline->spline_metadata_[i];

		spline_metadata.speed_limit = new_speed_limit_;

		CarPathSpline->spline_metadata_[i] = spline_metadata;
	}
}
void ABaseCarPath::makeCollMeshesAlongSpline()
{
	// iterate thru all coll meshes, destroy them and remove pointers
	for (int32 i = 0; i < spline_meshes_.Num(); i++)
	{
		if (spline_meshes_[i])
		{
			spline_meshes_[i]->DestroyComponent();

			if (spline_meshes_[i])
			{
				spline_meshes_[i]->SetActive(false);
				spline_meshes_[i]->UnregisterComponent();
			}
		}
	}

	if (use_spline_coll_mesh_ && coll_mesh_)
	{
		coll_mesh_spacing_ = coll_mesh_->GetBounds().BoxExtent.X * coll_mesh_scale_ * coll_mesh_spacing_scale_;
		coll_mesh_->GetBounds().BoxExtent.X;
		//coll_mesh_tangent_length_ = coll_mesh_tangent_length_ * coll_mesh_scale_ * coll_mesh_spacing_scale_;

		float local_mesh_tangent_length = coll_mesh_tangent_length_ * coll_mesh_scale_ * coll_mesh_spacing_scale_;
		coll_mesh_end_tangent_length_ = coll_mesh_tangent_length_ * coll_mesh_scale_ * coll_mesh_spacing_scale_;

		coll_mesh_num_ = CarPathSpline->GetSplineLength() / coll_mesh_spacing_;

		for (int32 i = 0; i <= coll_mesh_num_; i++)
		{
			class USplineMeshComponent* new_spline_mesh_comp = NewObject<USplineMeshComponent>(this);

			if (new_spline_mesh_comp)
			{
				new_spline_mesh_comp->SetMobility(EComponentMobility::Movable);
				new_spline_mesh_comp->SetStaticMesh(coll_mesh_);
				new_spline_mesh_comp->SetHiddenInGame(true);

				// NOTE: game MUST have this collision profile set up! or it won't work
				new_spline_mesh_comp->SetCollisionProfileName(FName("LaneChecker"));

				new_spline_mesh_comp->RegisterComponent();
				new_spline_mesh_comp->AttachTo(RootComponent);

				FVector start_location = CarPathSpline->GetLocationAtDistanceAlongSpline(i * coll_mesh_spacing_, ESplineCoordinateSpace::Local);
				FVector start_tangent = CarPathSpline->GetDirectionAtDistanceAlongSpline(i * coll_mesh_spacing_, ESplineCoordinateSpace::Local) * local_mesh_tangent_length;

				FVector end_location = CarPathSpline->GetLocationAtDistanceAlongSpline(i * coll_mesh_spacing_ + coll_mesh_spacing_, ESplineCoordinateSpace::Local);
				FVector end_tangent = CarPathSpline->GetDirectionAtDistanceAlongSpline(i * coll_mesh_spacing_ + coll_mesh_spacing_, ESplineCoordinateSpace::Local) * local_mesh_tangent_length;


				new_spline_mesh_comp->SetStartAndEnd(start_location, start_tangent, end_location, end_tangent, true);
				new_spline_mesh_comp->SetStartScale( FVector2D(coll_mesh_scale_, coll_mesh_scale_), true );
				new_spline_mesh_comp->SetEndScale(FVector2D(coll_mesh_scale_, coll_mesh_scale_), true);

				new_spline_mesh_comp->RegisterComponent();

				spline_meshes_.Add(new_spline_mesh_comp);

			}
		}
	}
}
