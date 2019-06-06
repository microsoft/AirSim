// Fill out your copyright notice in the Description page of Project Settings.
#include "NPCVehicleManager.h"
#include "BaseCarPath.h"
#include "BaseBicyclePath.h"
//#include "Materials/MaterialParameterCollectionInstance.h"
#include "Materials/MaterialParameterCollection.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"


//class ACarPawn;
// Sets default values
ANPCVehicleManager::ANPCVehicleManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root_comp_ = CreateDefaultSubobject<USceneComponent>(TEXT("RootComp"));

	HISM_cull_start_dist_ = 20000;
	HISM_cull_end_dist_ = 22000;

	default_num_cars_per_path_ = 2;

	default_num_bicycles_per_path_ = 1;

	debug_max_cars_ = -1;

}
TArray<AActor*> ANPCVehicleManager::getAllAirsimCarPawns()
{
	if (local_pawns_.Num() == 0)
	{
		// TODO: just hardcode these airsim pawns somewhere...
		if (!has_complained_about_no_airsim_vehicles_)
		{
			UE_LOG(LogTemp, Warning, TEXT("WARNING! AirsimCar pawns has an empty array! Be sure to override SetAirsimCarPawns() to set AirsimCarPawns to the airsim pawns!"));
			has_complained_about_no_airsim_vehicles_ = true;
		}
		
		TArray<AActor*> EmptyArray;
		EmptyArray.Add(NULL);
		return EmptyArray;
	}
	return local_pawns_;
}
int32 ANPCVehicleManager::getCurrentNumCarsPerPath()
{
	return current_num_cars_per_path_;
}
void ANPCVehicleManager::initSpawnCarsOnPaths(int32 OverrideNumCarsPerPath)
{
	destroyAllCars();
	// we need world to do this!!!
	if (!GetWorld())
	{
		UE_LOG(LogTemp, Warning, TEXT("COULD NOT GET WORLD!!"));
		return;
	}

	// not necessary if vehicle types are not set, save perf
	if (vehicle_types_.Num() == 0)
	{
		return;
	}

	for (int32 i = 0; i < vehicle_types_.Num(); i++)
	{
		UHierarchicalInstancedStaticMeshComponent *NewHISM = NewObject<UHierarchicalInstancedStaticMeshComponent>(this);
		if (NewHISM)
		{
			NewHISM->SetCullDistances(HISM_cull_start_dist_, HISM_cull_end_dist_);
			NewHISM->SetSingleSampleShadowFromStationaryLights(true);
			NewHISM->bReceiveCombinedCSMAndStaticShadowsFromStationaryLights = false;
			NewHISM->RegisterComponent();
			NewHISM->SetStaticMesh(vehicle_types_[i].static_mesh_);
			NewHISM->bGenerateOverlapEvents = true;

			// required for segementation view, otherwise it doesn't appear
			NewHISM->bRenderCustomDepth = true;
			// TODO: blueprint to override this in case we have different collision profiles
			// NOTE these should NOT collide with car paths, it will cause confusion because they dont know which instance is on which path...
			NewHISM->SetCollisionProfileName("NPCVehicle");
			instanced_mesh_comps_.Add(NewHISM);

			vehicle_types_[i].hism_ = NewHISM;
		}
	}

	for (int32 i = 0; i < bicycle_types_.Num(); i++)
	{
		UHierarchicalInstancedStaticMeshComponent *NewHISM = NewObject<UHierarchicalInstancedStaticMeshComponent>(this);
		if (NewHISM)
		{
			NewHISM->SetCullDistances(HISM_cull_start_dist_, HISM_cull_end_dist_);
			NewHISM->SetSingleSampleShadowFromStationaryLights(true);
			NewHISM->bReceiveCombinedCSMAndStaticShadowsFromStationaryLights = false;
			NewHISM->RegisterComponent();
			NewHISM->SetStaticMesh(bicycle_types_[i].static_mesh_);
			NewHISM->bGenerateOverlapEvents = true;

			// required for segementation view, otherwise it doesn't appear
			NewHISM->bRenderCustomDepth = true;
			// TODO: blueprint to override this in case we have different collision profiles
			// NOTE these should NOT collide with car paths, it will cause confusion because they dont know which instance is on which path...
			NewHISM->SetCollisionProfileName("NPCVehicle");
			instanced_mesh_comps_.Add(NewHISM);

			bicycle_types_[i].hism_ = NewHISM;
		}
	}

	int32 NumCarsPerPath = default_num_cars_per_path_;
	if (OverrideNumCarsPerPath != -1)
	{
		NumCarsPerPath = OverrideNumCarsPerPath;
	}
	current_num_cars_per_path_ = NumCarsPerPath;

	// get all base car paths
	TArray<AActor*> CarPathActors;

	if (GetWorld())
	{
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABaseCarPath::StaticClass(), CarPathActors);
	}
	if (use_car_headlights_)
	{
		turnOnAllCarLights();
	}

	// if we have no car path actors, dont do this!! it will crash!!
	if (CarPathActors.Num() > 0)
	{
		int32 NumCarPathActors = CarPathActors.Num();

		if (debug_only_one_car_path_) 
		{
			NumCarPathActors = 1;
		}
		for (int32 i = 0; i < NumCarPathActors; i++)
		{
			ABaseCarPath* CarPath = Cast<ABaseCarPath>(CarPathActors[i]);
			// bicycle path is a child class of carpath, don't put cars on bicycle paths!
			ABaseBicyclePath* BicyclePath = Cast<ABaseBicyclePath>(CarPathActors[i]);
			if (BicyclePath)
			{
				if (BicyclePath->getCarPathSpline())
				{
					// set the vehicle manager so it can get the airsim car info later, for checking collisions
					BicyclePath->MyVehicleManager = this;
					for (int32 j = 0; j < default_num_bicycles_per_path_; j++)
					{
						//debug limit max number of spawned cars
						/*if (debug_max_cars_ != -1 && total_num_vehicle_instances_ > debug_max_cars_)
						{
							return;
						}*/
						// just spawn a car at the beginning of the spline for now
						if (BicyclePath->spawnVehicle(getRandomBicycleType(), 0))
						{
							//total_num_vehicle_instances_++;
						}
					}
				}
			}
			else if (CarPath && CarPath->getCarPathSpline())
			{
				// set the vehicle manager so it can get the airsim car info later, for checking collisions
				CarPath->MyVehicleManager = this;
				for (int32 j = 0; j < NumCarsPerPath; j++)
				{
					//debug limit max number of spawned cars
					if (debug_max_cars_ != -1 && total_num_vehicle_instances_ > debug_max_cars_)
					{
						return;
					}
					// just spawn a car at the beginning of the spline for now
					if (CarPath->spawnVehicle(GetRandomVehicleType(), 0))
					{
						total_num_vehicle_instances_++;
					}

				}
			}
		}
	}
}

USpotLightComponent* ANPCVehicleManager::spawnCarLight()
{
	//if (use_car_headlights_)
	//{
		USpotLightComponent *NewLight = NewObject<USpotLightComponent>(this);
		if (NewLight)
		{
			NewLight->RegisterComponent();
			NewLight->SetCastShadows(false);
			NewLight->SetInnerConeAngle(50.0f);
			NewLight->SetOuterConeAngle(70.0f);

			//NewLight->SetIntensity(13000.0f);
			NewLight->SetIntensity(0.0f);

			car_lights_.Add(NewLight);
			return NewLight;
		}
	//}
	return NULL;
}
void ANPCVehicleManager::turnOnAllCarLights()
{
	if (GetWorld())
	{
		for (int32 i = 0; i < car_lights_.Num(); i++)
		{
			car_lights_[i]->SetIntensity(13000.0f);
		}
		UMaterialParameterCollection* TrafficParamCollection = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getTrafficParamsObjectPath()));

		//UWorld* World = GetWorld();
		if (TrafficParamCollection)
		{
			UMaterialParameterCollectionInstance* Instance = GetWorld()->GetParameterCollectionInstance(TrafficParamCollection);
			if (Instance)
			{
				//return Instance;
				Instance->SetScalarParameterValue(TEXT("CarLight"), 1.0f);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get TrafficParamCollection!"));
			}

		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get TrafficParamCollection!"));
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get World!"));
	}
}
void ANPCVehicleManager::turnOffAllCarLights()
{
	if (GetWorld())
	{
		for (int32 i = 0; i < car_lights_.Num(); i++)
		{
			car_lights_[i]->SetIntensity(0.0f);
		}
		UMaterialParameterCollection* TrafficParamCollection = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getTrafficParamsObjectPath()));

		//UWorld* World = GetWorld();
		if (TrafficParamCollection)
		{
			UMaterialParameterCollectionInstance* Instance = GetWorld()->GetParameterCollectionInstance(TrafficParamCollection);
			if (Instance)
			{
				//return Instance;
				Instance->SetScalarParameterValue(TEXT("CarLight"), 0.0f);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get TrafficParamCollection!"));
			}

		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get TrafficParamCollection!"));
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, could NOT get World!"));
	}
}
void ANPCVehicleManager::removeCarLight(USpotLightComponent* spot_light_)
{
	car_lights_.Remove(spot_light_);
}
ASimModeBase* ANPCVehicleManager::getMySimModeBase()
{
	// only get it once - once we have it, keep the pointer so we dont have to call GetAllActorsOfClass again (it's expensive)
	if (mySimModeBase_ == nullptr)
	{
		if (GetWorld())
		{
			TArray <AActor*> SimModeBaseActors;
			UGameplayStatics::GetAllActorsOfClass(GetWorld(), ASimModeBase::StaticClass(), SimModeBaseActors);

			if (SimModeBaseActors.IsValidIndex(0))
			{
				mySimModeBase_ = Cast<ASimModeBase>(SimModeBaseActors[0]);
			}
		}
	}

	return mySimModeBase_;
}
void ANPCVehicleManager::destroyAllCars()
{
	// get all base car paths
	TArray<AActor*> CarPathActors;

	if (GetWorld())
	{
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABaseCarPath::StaticClass(), CarPathActors);

		for (int32 i = 0; i < CarPathActors.Num(); i++)
		{
			ABaseCarPath* CarPath = Cast<ABaseCarPath>(CarPathActors[i]);
			if (CarPath)
			{
				CarPath->destroyVehicles();
			}
			
		}
	}
	total_num_vehicle_instances_ = 0;
	// go thru HISM, destroy everything
	for (int32 i = 0; i < instanced_mesh_comps_.Num(); i++)
	{
		instanced_mesh_comps_[i]->DestroyComponent();
	}
	instanced_mesh_comps_.Empty();
}
UHierarchicalInstancedStaticMeshComponent* ANPCVehicleManager::GetRandomVehicleInstancedMeshComp()
{
	if (instanced_mesh_comps_.Num() > 0)
	{
		for (int32 i = 0; i < instanced_mesh_comps_.Num(); i++)
		{
			int32 RandomIndex = FMath::RandRange(0, instanced_mesh_comps_.Num() - 1);

			return instanced_mesh_comps_[RandomIndex];
		}
	}
	return NULL;
}
FNPCVehicleType ANPCVehicleManager::GetRandomVehicleType()
{
	// NEW RANDOM CODE, that takes into account random perc chance
	// NOTE, this assumes VehicleTypes has the same indexes and num as InstancedMeshComps
	float TotalPercent = 0.0f;

	for (int32 i = 0; i < vehicle_types_.Num(); i++)
	{
		TotalPercent += vehicle_types_[i].perc_chance_to_spawn_;
	}

	float RandomPercent = FMath::FRandRange(0, TotalPercent);

	float CurrentIndexPercent = 0.0f;
	float NextIndexPercent = 0.0f;
	for (int32 i = 0; i < vehicle_types_.Num(); i++)
	{
		if (i < vehicle_types_.Num() - 1)
		{
			NextIndexPercent = CurrentIndexPercent + vehicle_types_[i].perc_chance_to_spawn_;
		}
		else
		{
			return vehicle_types_[i];
		}

		if (RandomPercent > CurrentIndexPercent && RandomPercent < NextIndexPercent)
		{
			return vehicle_types_[i];
		}

		CurrentIndexPercent += vehicle_types_[i].perc_chance_to_spawn_;
	}
	UE_LOG(LogTemp, Warning, TEXT("Something went wrong, returning null vehicle type"));
	FNPCVehicleType NullVehicle;
	return NullVehicle;
}
FNPCVehicleType ANPCVehicleManager::getRandomBicycleType()
{
	// NEW RANDOM CODE, that takes into account random perc chance
	// NOTE, this assumes VehicleTypes has the same indexes and num as InstancedMeshComps
	float TotalPercent = 0.0f;

	for (int32 i = 0; i < bicycle_types_.Num(); i++)
	{
		TotalPercent += bicycle_types_[i].perc_chance_to_spawn_;
	}

	float RandomPercent = FMath::FRandRange(0, TotalPercent);

	float CurrentIndexPercent = 0.0f;
	float NextIndexPercent = 0.0f;
	for (int32 i = 0; i < bicycle_types_.Num(); i++)
	{
		if (i < bicycle_types_.Num() - 1)
		{
			NextIndexPercent = CurrentIndexPercent + vehicle_types_[i].perc_chance_to_spawn_;
		}
		else
		{
			return bicycle_types_[i];
		}

		if (RandomPercent > CurrentIndexPercent && RandomPercent < NextIndexPercent)
		{
			return bicycle_types_[i];
		}

		CurrentIndexPercent += bicycle_types_[i].perc_chance_to_spawn_;
	}
	UE_LOG(LogTemp, Warning, TEXT("Something went wrong, returning null vehicle type"));
	FNPCVehicleType NullVehicle;
	return NullVehicle;
}
// Called when the game starts or when spawned
void ANPCVehicleManager::BeginPlay()
{
	Super::BeginPlay();

	initSpawnCarsOnPaths();
}

// Called every frame
void ANPCVehicleManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// mark all render state dirty - forces all the instances to update rendering
	for (int32 i = 0; i < instanced_mesh_comps_.Num(); i++)
	{
		FTransform InstanceTransform;
		instanced_mesh_comps_[i]->GetInstanceTransform(0, InstanceTransform, true);
		//int32 InstanceIndex, const FTransform& NewInstanceTransform, bool bWorldSpace, bool bMarkRenderStateDirty = false, bool bTeleport = false
		instanced_mesh_comps_[i]->UpdateInstanceTransform(0, InstanceTransform, true, true, true);
	}

}

