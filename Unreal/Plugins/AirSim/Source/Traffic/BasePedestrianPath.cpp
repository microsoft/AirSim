// Fill out your copyright notice in the Description page of Project Settings.

//#include "Kismet\KismetSystemLibrary.h"
#include "BasePedestrianPath.h"


// Sets default values
ABasePedestrianPath::ABasePedestrianPath()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	bCanSpawnPedestrians = true;

	SplineCollSphereRadius = 50.0f;
	SplineCollSphereSpacing = 100.0f;

	end_sphere_coll_radius = 500.0f;

	spline_comp_ = CreateDefaultSubobject<USplineComponent>(TEXT("SplineComp"));
	start_sphere_ = CreateDefaultSubobject<USphereComponent>(TEXT("start_sphere"));
	start_sphere_->SetCollisionProfileName(FName("OverlapAll"));
	start_sphere_->SetSphereRadius(end_sphere_coll_radius);
	end_sphere_ = CreateDefaultSubobject<USphereComponent>(TEXT("end_sphere"));
	end_sphere_->SetCollisionProfileName(FName("OverlapAll"));
	end_sphere_->SetSphereRadius(end_sphere_coll_radius);

	width_ = 200.0f;

	spline_mesh_spacing_ = 200.0f;

	spline_mesh_scale_ = FVector(2, 2, 0.1);
}
void ABasePedestrianPath::OnConstruction(const FTransform & Transform)
{
	Super::OnConstruction(Transform);

	SetVisualizationWidth(width_);

	for (int32 i = 0; i < spline_meshes_.Num(); i++)
	{
		if (spline_meshes_[i])
		{
			spline_meshes_[i]->DestroyComponent();
			if (spline_meshes_[i])
			{
				spline_meshes_[i]->UnregisterComponent();
				spline_meshes_[i]->SetActive(false);
			}
		}
	}
	spline_meshes_.Empty();

	int32 num_meshes = spline_comp_->GetSplineLength() / spline_mesh_spacing_;
	float dist_on_spline;

	for (int32 i = 0; i <= num_meshes; i++)
	{
		dist_on_spline = spline_mesh_spacing_ * i;

		FVector new_mesh_location = spline_comp_->GetLocationAtDistanceAlongSpline(dist_on_spline, ESplineCoordinateSpace::World);
		FRotator new_mesh_rotation = spline_comp_->GetRotationAtDistanceAlongSpline(dist_on_spline, ESplineCoordinateSpace::World);

		UStaticMeshComponent* new_mesh = NewObject<UStaticMeshComponent>(this);
		if (new_mesh)
		{
			// prevents garbage collection during runtime
			new_mesh->RegisterComponent();
			new_mesh->AttachTo(RootComponent);
			new_mesh->SetRelativeScale3D(spline_mesh_scale_);
			new_mesh->SetStaticMesh(spline_mesh_);
			new_mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

			new_mesh->SetHiddenInGame(true);

			new_mesh->SetWorldLocation(new_mesh_location);
			new_mesh->SetWorldRotation(new_mesh_rotation);

			spline_meshes_.Add(new_mesh);
		}
	}

	// start and end spheres check for collisions with other ped paths, so NPCs will try to walk to those when finished this one.
	start_sphere_->SetWorldLocation(spline_comp_->GetLocationAtDistanceAlongSpline(0.0f, ESplineCoordinateSpace::World));
	end_sphere_->SetWorldLocation(spline_comp_->GetLocationAtDistanceAlongSpline(spline_comp_->GetSplineLength(), ESplineCoordinateSpace::World));

	// spawn coll spheres
	if (bUseCollSpheres)
	{
		for (int32 i = 0; i < CollSpheres.Num(); i++)
		{
			if (CollSpheres[i])
			{
				CollSpheres[i]->DestroyComponent();
			}
		}
		CollSpheres.Empty();
			int32 MaxNumCollSpheres = 50;
			int32 NumCollSpheres = FMath::Clamp(static_cast<int32>(spline_comp_->GetSplineLength() / SplineCollSphereSpacing), 0, MaxNumCollSpheres);

		for (int32 j = 0; j < NumCollSpheres; j++)
		{
			class UBaseCarPathCollSphereComponent* NewSphereComp = NewObject<UBaseCarPathCollSphereComponent>(this);
			if (NewSphereComp)
			{

				FTransform NewSphereTransform = spline_comp_->GetTransformAtDistanceAlongSpline(j * SplineCollSphereSpacing, ESplineCoordinateSpace::World);
				NewSphereComp->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
				NewSphereComp->SetSphereRadius(SplineCollSphereRadius, false);
				NewSphereComp->SetWorldLocation(NewSphereTransform.GetLocation());
				// by default no collisions. we only enable them when we want to test for collision
				// TODO: blueprint to override this in case we have different collision profiles
				NewSphereComp->SetCollisionProfileName("CarPathCollSphere");
				// make sure to do this last, setting collision profile will override this!!
				NewSphereComp->disableCollision();

				NewSphereComp->bGenerateOverlapEvents = true;
				//NewSphereComp->Overlap
			}


			CollSpheres.Add(NewSphereComp);
		}
	}

	// end connecting spheres for checking connecting paths

	NextPedPath.Empty();
	PrevPedPath.Empty();

	TArray<AActor*> start_sphere_overlapping_actors;
	start_sphere_->GetOverlappingActors(start_sphere_overlapping_actors, ABasePedestrianPath::StaticClass());

	//UKismetSystemLibrary::ComponentOverlapActors(start_sphere_, start_sphere_->GetComponentTransform(), const TArray<TEnumAsByte<EObjectTypeQuery> > & ObjectTypes, UClass* ActorClassFilter, const TArray<AActor*>& ActorsToIgnore, TArray<AActor*>& OutActors)

	TArray<FOverlapResult> start_sphere_overlaps;

	GetWorld()->ComponentOverlapMulti(start_sphere_overlaps, start_sphere_, start_sphere_->GetComponentLocation(), start_sphere_->GetComponentRotation().Quaternion());//, Params, ObjectParams);
	//bool ComponentOverlapMulti(TArray<struct FOverlapResult>& OutOverlaps, const class UPrimitiveComponent* PrimComp, const FVector& Pos, const FQuat& Rot, const FComponentQueryParams& Params = FComponentQueryParams::DefaultComponentQueryParams, const FCollisionObjectQueryParams& ObjectQueryParams = FCollisionObjectQueryParams::DefaultObjectQueryParam) const;



	//for (int32 i = 0; i < start_sphere_overlapping_actors.Num(); i++)
	for (int32 i = 0; i < start_sphere_overlaps.Num(); i++)
	{
		//if (start_sphere_overlapping_actors[i] != this)
		if (start_sphere_overlaps[i].Actor != this)
		{
			//ABasePedestrianPath* test_path = Cast<ABasePedestrianPath>(start_sphere_overlapping_actors[i]);
			ABasePedestrianPath* test_path = Cast<ABasePedestrianPath>(start_sphere_overlaps[i].Actor.Get());
			if (test_path)
			{
				float dist = FVector::Dist(start_sphere_->GetComponentLocation(), test_path->start_sphere_->GetComponentLocation());
				if (dist <= (start_sphere_->GetScaledSphereRadius() * 2))
				{
					PrevPedPath.Add(FConnectingPedPathInfo(test_path, true));
				}
				else
				{
					PrevPedPath.Add(FConnectingPedPathInfo(test_path, false));
				}
			}
		}
	}

	//TArray<AActor*> end_sphere_overlapping_actors;

	TArray<FOverlapResult> end_sphere_overlaps;
	//end_sphere_->GetOverlappingActors(end_sphere_overlapping_actors, ABasePedestrianPath::StaticClass());

	GetWorld()->ComponentOverlapMulti(end_sphere_overlaps, end_sphere_, end_sphere_->GetComponentLocation(), end_sphere_->GetComponentRotation().Quaternion());//, Params, ObjectParams);


	//for (int32 i = 0; i < end_sphere_overlapping_actors.Num(); i++)
	for (int32 i = 0; i < end_sphere_overlaps.Num(); i++)
	{
		//if (end_sphere_overlapping_actors[i] != this)
		if (end_sphere_overlaps[i].Actor != this)
		{
			ABasePedestrianPath* test_path = Cast<ABasePedestrianPath>(end_sphere_overlaps[i].Actor.Get());
			if (test_path)
			{
				float dist = FVector::Dist(end_sphere_->GetComponentLocation(), test_path->end_sphere_->GetComponentLocation());
				if (dist <= (end_sphere_->GetScaledSphereRadius() * 2))
				{
					NextPedPath.Add(FConnectingPedPathInfo(test_path, true));
				}
				else
				{
					NextPedPath.Add(FConnectingPedPathInfo(test_path, false));
				}
			}
		}
	}
}
FConnectingPedPathInfo ABasePedestrianPath::GetRandomConnectingPath(bool bUseRear)
{
	int32 rand_index;
	FConnectingPedPathInfo null_struct;
	if (bUseRear)
	{
		rand_index = FMath::RandRange(0, PrevPedPath.Num()-1);
		if (PrevPedPath.Num() > 0)
		{
			return PrevPedPath[rand_index];
		}
		else
		{
			return null_struct;
		}
	}
	else
	{
		rand_index = FMath::RandRange(0, NextPedPath.Num()-1);
		if (NextPedPath.Num() > 0)
		{
			return NextPedPath[rand_index];
		}
		else
		{
			return null_struct;
		}
	}
	return null_struct;
}
bool ABasePedestrianPath::GetIsWalkable()
{
	if (force_walking_disabled_)
	{
		return false;
	}
	if (traffic_status_ == ETrafficStatus::TRAFFIC_GREEN)
	{
		return true;
	}
	return false;
}
/*void ABasePedestrianPath::disableAllCollSpheres()
{
	for (int32 i = 0; i < CollSpheres.Num(); i++)
	{
		//CollSpheres[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		CollSpheres[i]->disableCollision();
	}
}*/
void ABasePedestrianPath::enableCollSpheres(float Distance, bool bReverse)
{
	int32 StartingSphereIndex = Distance / SplineCollSphereSpacing;

	// for loop will crash if this happens
	if (StartingSphereIndex > CollSpheres.Num() - 1)
	{
		return;
	}

	// how many spheres to enable ahead.
	// TODO: a more intelligent way of counting these would be nice
	int32 NumAheadSpheres = 4;
	// if reverse, count backwards
	if (bReverse)
	{
		NumAheadSpheres *= -1;
	}

	int32 EndSphere = FMath::Clamp((StartingSphereIndex + NumAheadSpheres), 0, CollSpheres.Num() - 1);



	if (bReverse)
	{
		StartingSphereIndex = FMath::Clamp((StartingSphereIndex + 1), 0, CollSpheres.Num() - 1);
		for (int32 i = StartingSphereIndex; i > EndSphere; i--)
		{
			// TODO: perhaps intelligent autodelay, based on how fast ped is walking?
			CollSpheres[i]->enableCollision(1.0f);
		}
	}
	else
	{
		for (int32 i = StartingSphereIndex; i < EndSphere; i++)
		{
			// TODO: perhaps intelligent autodelay, based on how fast ped is walking?
			CollSpheres[i]->enableCollision(1.0f);
		}
	}

}
// Called when the game starts or when spawned
void ABasePedestrianPath::BeginPlay()
{
	Super::BeginPlay();

	/*NextPedPath.Empty();
	PrevPedPath.Empty();

	TArray<AActor*> start_sphere_overlapping_actors;
	start_sphere_->GetOverlappingActors(start_sphere_overlapping_actors, ABasePedestrianPath::StaticClass());

	for (int32 i = 0; i < start_sphere_overlapping_actors.Num(); i++)
	{
		if (start_sphere_overlapping_actors[i] != this)
		{
			ABasePedestrianPath* test_path = Cast<ABasePedestrianPath>(start_sphere_overlapping_actors[i]);
			if (test_path)
			{
				float dist = FVector::Dist(start_sphere_->GetComponentLocation(), test_path->start_sphere_->GetComponentLocation());
				if (dist <= (start_sphere_->GetScaledSphereRadius() * 2))
				{
					PrevPedPath.Add(FConnectingPedPathInfo(test_path, true));
				}
				else
				{
					PrevPedPath.Add(FConnectingPedPathInfo(test_path, false));
				}
			}
		}
	}

	TArray<AActor*> end_sphere_overlapping_actors;
	end_sphere_->GetOverlappingActors(end_sphere_overlapping_actors, ABasePedestrianPath::StaticClass());

	for (int32 i = 0; i < end_sphere_overlapping_actors.Num(); i++)
	{
		if (end_sphere_overlapping_actors[i] != this)
		{
			ABasePedestrianPath* test_path = Cast<ABasePedestrianPath>(end_sphere_overlapping_actors[i]);
			if (test_path)
			{
				float dist = FVector::Dist(end_sphere_->GetComponentLocation(), test_path->end_sphere_->GetComponentLocation());
				if (dist <= (end_sphere_->GetScaledSphereRadius() * 2))
				{
					NextPedPath.Add(FConnectingPedPathInfo(test_path, true));
				}
				else
				{
					NextPedPath.Add(FConnectingPedPathInfo(test_path, false));
				}
			}
		}
	}

	// disable those collisions after 5 seconds, making sure they're all initialized
	GetWorld()->GetTimerManager().SetTimer(disable_end_spheres_timer, this, &ABasePedestrianPath::disableEndSpheres, 5.0f, false);*/
	
}

void ABasePedestrianPath::SetVisualizationWidth(float Width)
{

	if (spline_comp_)
	{
		// packaging doesnt like this unfortunately
		//SplineComp->ScaleVisualizationWidth = Width;
	}

}

// Called every frame
void ABasePedestrianPath::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}
USplineComponent* ABasePedestrianPath::getSplineComp()
{
	return spline_comp_;
}
void ABasePedestrianPath::disableEndSpheres()
{
	start_sphere_->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	end_sphere_->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}