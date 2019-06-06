// Fill out your copyright notice in the Description page of Project Settings.

#include "NPCPedestrianManager.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"


// Sets default values
ANPCPedestrianManager::ANPCPedestrianManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	default_num_pedestrians_per_spline_ = 5;

	walk_speed_min_ = 75.0f;
	walk_speed_max_ = 200.0f;

	rand_offset_min_ = 0.0f;
	rand_offset_max_ = 150.0f;

	max_draw_distance_ = 15000.0f;
	is_collisions_enabled_ = true;

	debug_max_peds_ = -1;

	MaxDistCollCheck = 15000.0f;

	//SetFlags(EObjectFlags::RF_MarkAsRootSet);
}

// Called when the game starts or when spawned
void ANPCPedestrianManager::BeginPlay()
{
	Super::BeginPlay();

	spawnPedestrians();
	
}
TArray<AActor*> ANPCPedestrianManager::getAllAirsimCarPawns()
{
	return local_pawns_;
}
// Called every frame
void ANPCPedestrianManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	updatePedestrians(DeltaTime);

}
void ANPCPedestrianManager::spawnPedestrians(int32 OverrideNumPedestriansPerSpline)
{
	// no peds at all, dont bother, may crash
	if (pedestrian_archetypes_.Num() == 0)
	{
		return;
	}
	for (int32 i = 0; i < pedestrians_.Num(); i++)
	{
		//SkelMeshes[i]->DestroyComponent();
		if (pedestrians_[i].skel_mesh_comp_ != nullptr)
		{
			pedestrians_[i].skel_mesh_comp_->DestroyComponent();
		}
	}
	pedestrians_.Empty();

	coll_enabled_ped_paths_.Empty();

	// get all base car paths
	TArray<AActor*> PedestrianPathActors;
	if (GetWorld())
	{
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABasePedestrianPath::StaticClass(), PedestrianPathActors);
	}
	// now spawn the appropriate amount of skel meshes on the splines

	int32 NumPedestriansPerSpline = default_num_pedestrians_per_spline_;
	if (OverrideNumPedestriansPerSpline != -1)
	{
		NumPedestriansPerSpline = OverrideNumPedestriansPerSpline;
	}
	current_num_pedestrians_per_spline_ = NumPedestriansPerSpline;
	int32 NumPedActors = PedestrianPathActors.Num();

	if (debug_only_one_spline_ && PedestrianPathActors.Num() > 0)
	{
		NumPedActors = 1;
	}

	total_num_spawned_peds_ = 0;
	for (int32 i = 0; i < PedestrianPathActors.Num(); i++)
	{
		//PedestrianPathActors
		ABasePedestrianPath* PedPath = Cast<ABasePedestrianPath>(PedestrianPathActors[i]);

		if (PedPath && PedPath->bUseCollSpheres)
		{
			coll_enabled_ped_paths_.Add(PedPath);
		}
		// perhaps mod 2 == 0, so we only spawn on half the roads?
		// only on even roads for now, reducing amount of peds without looking too sparse
		if (PedPath && PedPath->bCanSpawnPedestrians )// && (i % 2 == 0) )
		{
			for (int32 j = 0; j < NumPedestriansPerSpline; j++)
			{

				// HACK disable spawning if limited by debug
				if (debug_max_peds_ != -1)
				{
					if (total_num_spawned_peds_ > debug_max_peds_)
					{
						return;
					}
				}
				float SplineLength = PedPath->getSplineComp()->GetSplineLength();
				float DistPerPed = FMath::FloorToFloat(SplineLength / NumPedestriansPerSpline);

				FPedestrianInfo NewPedestrian;
				NewPedestrian.skel_mesh_comp_ = NewObject<USkeletalMeshComponent>(this);
				NewPedestrian.skel_mesh_comp_->LDMaxDrawDistance = max_draw_distance_;

				//required to show up in segmentation view
				NewPedestrian.skel_mesh_comp_->bRenderCustomDepth = true;

				// not sure what this doeas
				//NewPedestrian.SkelMeshComp->bComponentUseFixedSkelBounds = true;

				float RandomMultiplier = FMath::RandRange(0.3f, 1.0f);

				NewPedestrian.dist_on_spline_ = (DistPerPed * RandomMultiplier) * j;

				NewPedestrian.skel_mesh_comp_->RegisterComponent();

				int32 RandomPedMeshIndex = FMath::RandRange(0, pedestrian_archetypes_.Num() - 1);
				NewPedestrian.skel_mesh_comp_->SetSkeletalMesh(pedestrian_archetypes_[RandomPedMeshIndex].skel_mesh_);

				NewPedestrian.skel_mesh_comp_->bEnableUpdateRateOptimizations = true;


				NewPedestrian.skel_mesh_comp_->AnimUpdateRateParams->UpdateRate = 5;

				// basically dont update at all when not rendered
				NewPedestrian.skel_mesh_comp_->AnimUpdateRateParams->BaseNonRenderedUpdateRate = BaseNonRenderedUpdateRate;

				//rand bucket so they dont all update on the same frame
				EUpdateRateShiftBucket RandBucket = EUpdateRateShiftBucket::ShiftBucket0;

				int32 RandInt = FMath::RandRange(0, 5);
				switch (RandInt)
				{
				case 0:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket0;
				}
				case 1:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket1;
				}
				case 2:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket2;
				}
				case 3:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket3;
				}
				case 4:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket4;
				}
				case 5:
				{
					RandBucket = EUpdateRateShiftBucket::ShiftBucket5;
				}
				}
				// assign random buckets 
				NewPedestrian.skel_mesh_comp_->AnimUpdateRateParams->ShiftBucket = RandBucket;


				NewPedestrian.skel_mesh_comp_->AnimUpdateRateParams->BaseVisibleDistanceFactorThesholds.Empty();
				NewPedestrian.skel_mesh_comp_->AnimUpdateRateParams->BaseVisibleDistanceFactorThesholds = BaseVisibleDistanceFactorThesholds;

				// dont allow it to tick
				// TODO: perhaps find a solution where we tick it manually here?
				if (bDisableSkelMeshTick)
				{
					NewPedestrian.skel_mesh_comp_->SetComponentTickEnabled(false);
				}



				NewPedestrian.archetype_ = pedestrian_archetypes_[RandomPedMeshIndex];
				NewPedestrian.current_walk_speed_ = FMath::FRandRange(walk_speed_min_, walk_speed_max_);
				NewPedestrian.spline_offset_x_ = FMath::RandRange(rand_offset_min_, rand_offset_max_);
				NewPedestrian.spline_offset_y_ = FMath::RandRange(rand_offset_min_, rand_offset_max_);

				NewPedestrian.current_ped_path_ = PedPath;

				total_num_spawned_peds_++;
				pedestrians_.Add(NewPedestrian);
			}
		}
	}
}

void ANPCPedestrianManager::updatePedestrians(float DeltaTime)
{
	for (int32 i = 0; i < pedestrians_.Num(); i++)
	{
		//Pedestrians[i].SkelMeshComp->TickAnimation(DeltaTime, false);
		// check if close to airsim car, if so and collisions enabled then turn collisions on. otherwise turn off
		if (is_collisions_enabled_)
		{
			if (local_pawns_.Num() > 0)
			{
				bool bFoundCollision = false;
				for (int32 j = 0; j < local_pawns_.Num(); j++)
				{
					float Dist = FVector::Dist(local_pawns_[j]->GetActorLocation(), pedestrians_[i].skel_mesh_comp_->GetComponentLocation());
					
					// likely not to be a car smaller than this...
					// neighbourhood hack: we have fewer pedestrians, so we can afford more collisiosn from far away
					// this is necessary for j walking pedestrians also
					//float MaxDistCollCheck = 8000.0f;//2500.0f;
					if (Dist < MaxDistCollCheck && pedestrians_[i].skel_mesh_comp_->GetCollisionEnabled() == ECollisionEnabled::NoCollision)
					{
						//UE_LOG(LogTemp, Warning, TEXT("close enough, enabling collisions!!!!"));
						bFoundCollision = true;
						pedestrians_[i].skel_mesh_comp_->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
						break;
					}
				}
				if (!bFoundCollision && pedestrians_[i].skel_mesh_comp_->GetCollisionEnabled() == ECollisionEnabled::QueryAndPhysics)
				{
					pedestrians_[i].skel_mesh_comp_->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				}
			}
			else
			{
				if (!has_complained_about_no_airsim_vehicles_)
				{
					UE_LOG(LogTemp, Warning, TEXT("ped manager could not get any airsim car pawns!!!"));
					has_complained_about_no_airsim_vehicles_ = true;
				}
				
			}
		}

		bool bWasRenderedRecently = true;
		float RenderTimeTolerance = 0.1f;
		if (GetWorld() && GetWorld()->GetTimeSeconds() > (pedestrians_[i].skel_mesh_comp_->LastRenderTimeOnScreen + RenderTimeTolerance) )
		{
			//UE_LOG(LogTemp, Warning, TEXT("%d ped is not being rendered!!!"), i);
			pedestrians_[i].skel_mesh_comp_->SetComponentTickEnabled(false);
		}
		else
		{
			pedestrians_[i].skel_mesh_comp_->SetComponentTickEnabled(true);
			bWasRenderedRecently = true;
		}

		// check for changing paths
		if (pedestrians_[i].current_ped_path_ && bWasRenderedRecently)
		{
			// if we're going forward and we reached the end of spline
			// or going backwards and we reached beginning of spline
			// then we must move to next path
			if ((!pedestrians_[i].is_in_between_paths_ && pedestrians_[i].dist_on_spline_ == pedestrians_[i].current_ped_path_->getSplineComp()->GetSplineLength() && !pedestrians_[i].is_spline_reverse_ && pedestrians_[i].current_ped_path_->GetIsWalkable() /*&& Pedestrians[i].NextPedPath*/)
			|| (!pedestrians_[i].is_in_between_paths_ && pedestrians_[i].dist_on_spline_ == 0.0f && pedestrians_[i].is_spline_reverse_ && pedestrians_[i].current_ped_path_->GetIsWalkable() /*&& Pedestrians[i].NextPedPath*/))
			{
				//UE_LOG(LogTemp, Warning, TEXT("changing paths!!"));
				// we reached the end of the spline, we need to check if we can walk onto another path
				pedestrians_[i].last_ped_path_location_ = pedestrians_[i].skel_mesh_comp_->GetComponentLocation();


				FConnectingPedPathInfo ConnectingPathInfo = pedestrians_[i].current_ped_path_->GetRandomConnectingPath(false);
				// if we're going backwards, then we must've reached the rear end, so force us to move there instead
				if (pedestrians_[i].is_spline_reverse_)
				{
					ConnectingPathInfo = pedestrians_[i].current_ped_path_->GetRandomConnectingPath(true);
				}

				// make sure we have a valid path and spline comp first, otherwise it'll crash
				// theres a chance the coll spheres didnt get anything, or designer intentionally didnt have any connecting paths
				if (ConnectingPathInfo.ConnectingPath && ConnectingPathInfo.ConnectingPath->getSplineComp())
				{
					pedestrians_[i].next_ped_path_ = ConnectingPathInfo.ConnectingPath;

					if (ConnectingPathInfo.bConnectAt0)
					{
						pedestrians_[i].dist_on_spline_ = 0;
						pedestrians_[i].is_spline_reverse_ = false;
						//UE_LOG(LogTemp, Warning, TEXT("%f CONNECT AT ZERO CHANGING at dest between paths!!"), pedestrians_[i].dist_on_spline_);
					}
					else
					{
						pedestrians_[i].is_spline_reverse_ = true;
						pedestrians_[i].dist_on_spline_ = pedestrians_[i].next_ped_path_->getSplineComp()->GetSplineLength();
						//UE_LOG(LogTemp, Warning, TEXT("%f CHANGING at dest between paths!!"), pedestrians_[i].dist_on_spline_);

					}
					//UE_LOG(LogTemp, Warning, TEXT("setting in between paths to true!!"));
					pedestrians_[i].is_in_between_paths_ = true;

				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("no connecting path!!"));
					// no connecting path, just walk other way
					if (pedestrians_[i].is_spline_reverse_)
					{
						pedestrians_[i].is_spline_reverse_ = false;
					}
					else
					{
						pedestrians_[i].is_spline_reverse_ = true;
					}
				}
				PedestrianPlayAnim(i, pedestrians_[i].archetype_.idle_anim_, true);
			}
			// otherwise keep moving, either on my current path or towards next path
			else
			{
				FVector NewLocation = pedestrians_[i].skel_mesh_comp_->GetComponentLocation();;
				FRotator IdealRotation = pedestrians_[i].skel_mesh_comp_->GetComponentRotation();
				FVector OffsetVector = FVector(pedestrians_[i].spline_offset_x_, pedestrians_[i].spline_offset_y_, 0.0f);

				// move towards next path
				if (pedestrians_[i].is_in_between_paths_ /*&& Pedestrians[i].NextPedPath && Pedestrians[i].NextPedPath->GetIsWalkable()*/)
				{
					//UE_LOG(LogTemp, Warning, TEXT("between paths!!"));

					if (pedestrians_[i].next_ped_path_ && pedestrians_[i].next_ped_path_->GetIsWalkable())
					{
						FVector StartLoc = pedestrians_[i].skel_mesh_comp_->GetComponentLocation();
						// TODO: if in reverse, get the last point on the current ped path spline
						FVector Dest = pedestrians_[i].next_ped_path_->getSplineComp()->GetLocationAtDistanceAlongSpline(pedestrians_[i].dist_on_spline_, ESplineCoordinateSpace::World) + OffsetVector;

						if (StartLoc.Equals(Dest, 1.0f))
						{
							pedestrians_[i].is_in_between_paths_ = false;
							NewLocation = Dest;
							pedestrians_[i].current_ped_path_ = pedestrians_[i].next_ped_path_;
							pedestrians_[i].next_ped_path_ = NULL;

							//UE_LOG(LogTemp, Warning, TEXT("at dest between paths!!"));
						}
						else
						{
							float Alpha = 0.0f;
							float TotalDist = (StartLoc - Dest).Size();
							Alpha = (pedestrians_[i].current_walk_speed_ * DeltaTime) / TotalDist;

							Alpha = FMath::Clamp(Alpha, 0.0f, 1.0f);

							NewLocation = FMath::Lerp(StartLoc, Dest, Alpha);
							//UE_LOG(LogTemp, Warning, TEXT("moving between paths!!"));

							IdealRotation = UKismetMathLibrary::FindLookAtRotation(StartLoc, Dest);

						}
					}
					else
					{
						PedestrianPlayAnim(i, pedestrians_[i].archetype_.idle_anim_, true);
					}

					
				}
				// not between paths, just regular walking
				else
				{
					// just regular walking on the spline
					if (pedestrians_[i].is_spline_reverse_)
					{
						pedestrians_[i].dist_on_spline_ -= pedestrians_[i].current_walk_speed_ * DeltaTime;
					}
					else
					{
						pedestrians_[i].dist_on_spline_ += pedestrians_[i].current_walk_speed_ * DeltaTime;
					}

					if (pedestrians_[i].current_ped_path_->bUseCollSpheres)
					{

						pedestrians_[i].current_ped_path_->enableCollSpheres(pedestrians_[i].dist_on_spline_, pedestrians_[i].is_spline_reverse_);
					}
					
					// clamp it so it wont go to crazy values
					pedestrians_[i].dist_on_spline_ = FMath::Clamp(pedestrians_[i].dist_on_spline_, 0.0f, pedestrians_[i].current_ped_path_->getSplineComp()->GetSplineLength());
					//UE_LOG(LogTemp, Warning, TEXT("%f MOVING!!!"), Pedestrians[i].DistOnSpline);

					NewLocation = (pedestrians_[i].current_ped_path_->getSplineComp()->GetLocationAtDistanceAlongSpline(pedestrians_[i].dist_on_spline_, ESplineCoordinateSpace::World)) + OffsetVector;

					IdealRotation = pedestrians_[i].current_ped_path_->getSplineComp()->GetRotationAtDistanceAlongSpline(pedestrians_[i].dist_on_spline_, ESplineCoordinateSpace::World);

					// we need a way to check if location/rotation is valid or not, so we dont put him somewhere that isnt valid
					if (pedestrians_[i].is_spline_reverse_)
					{
						IdealRotation = UKismetMathLibrary::ComposeRotators(IdealRotation, FRotator(0.0f, 180.0f, 0.0f));
					}
				}

				FRotator NewRotation = FMath::RInterpConstantTo(pedestrians_[i].skel_mesh_comp_->GetComponentRotation(), IdealRotation, DeltaTime, pedestrians_[i].rotate_speed_);
				
				PedestrianPlayAnim(i, pedestrians_[i].archetype_.walking_anim_, true);

				// UPDATE LOC AND ROT HERE!!!
				pedestrians_[i].skel_mesh_comp_->SetWorldLocationAndRotation(NewLocation, NewRotation);
			}


		}
		else
		{
			
		}
	}
}

void ANPCPedestrianManager::PedestrianPlayAnim(int32 Index, UAnimationAsset* Animation, bool Loop)
{
	if (pedestrians_[Index].current_anim_ != Animation)
	{
		if (Animation != NULL)
		{
			pedestrians_[Index].skel_mesh_comp_->PlayAnimation(Animation, Loop);
			//Pedestrians[Index].SkelMeshComp->TickAnimation
			pedestrians_[Index].current_anim_ = Animation;
		}
		else
		{
			pedestrians_[Index].current_anim_ = NULL;
			pedestrians_[Index].skel_mesh_comp_->Stop();
		}

	}


}
int32 ANPCPedestrianManager::getCurrentNumPedsPerSpline()
{
	return current_num_pedestrians_per_spline_;
}