// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
//#include "BaseCarPath.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SpotLightComponent.h"
#include "SimMode/SimModeBase.h"
#include "NPCVehicleManager.generated.h"

// info for individual vehicle
// car paths keep an array of this and update it every frame
USTRUCT(BlueprintType)
struct FNPCVehicleInstance
{
	GENERATED_USTRUCT_BODY()

	// pointer to instanced mesh that we are an instance of
	// is this necessary??? maybe an index is enough
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = NPCVehicleInfo)
	UHierarchicalInstancedStaticMeshComponent* instanced_mesh_comp_;

	// instance of the instanced static mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = NPCVehicleInfo)
	int32 instance_index_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = NPCVehicleInfo)
	float dist_along_spline_;

	// by default is -1. if set to anything else, it is index of the connecting spline to go on. 
	// currently, it only sets when we reach the end of car path spline and collision checks allow us to start moving
	// so we use this index to determine if we are on the normal spline or connecting spline
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = NPCVehicleInfo)
	int32 connecting_spline_index_ = -1;

	// if true, we are moving on the connecting spline. otherwise we are on the main carpathspline
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category= NPCVehicleInfo)
	bool is_on_connecting_spline_;

	// TODO: if someday we require signaling lights from the vehicles, we will need to decide a connecting spline
	// before we reach the end of spline, and then move us onto the spline. so we need two ConnectingSpline variables for that

	// TODO: max speed and current speed
	// TODO: get from spline meta data
	UPROPERTY()
	float current_speed_;

	// roughly 40 mph. TODO get from spline metadata!
	// 1 mile == 160934 cm
	// SPEED FORMULA: 1mph = 160934cm /60/60 == 44.704
	UPROPERTY()
	float max_speed_ = 1800.0f;

	UPROPERTY()
	float acceleration_rate_;

	// deceleration rate is acceleration rate * 1.5
	//UPROPERTY()
	//float DecelerationRate;

	// pick a random speed varation value on spawn
	UPROPERTY()
	float current_speed_variation_;

	// speed variation, 0.0 is no variation, 1 is 100%
	UPROPERTY()
	float max_speed_variation_ = 0.2f;

	// UNUSED
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = NPCVehicleInfo)
	FRotator LastRotation;

	// currently unused - could be used to adjust stopping distances
	float car_length_;

	// spotlight used for night lights
	UPROPERTY()
	USpotLightComponent* spot_light_;

	// from the vehicle's 0,0,0
	UPROPERTY()
	FVector spot_light_offset_;

	// we need location maybe, for lerping?

	FNPCVehicleInstance()
	{
		connecting_spline_index_ = -1;
		instance_index_ = -1;
		dist_along_spline_ = 0.0f;
		is_on_connecting_spline_ = false;
		acceleration_rate_ = 380.0f;
		max_speed_ = 1800.0f;
		current_speed_ = 0.0f;
		spot_light_ = NULL;
		//DecelerationRate = AccelerationRate * 1.5f;
		//InstancedMeshComp = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedMeshComp"));
	}

	float getDecelerationRate()
	{
		// looks good enough for now, but later on could use a real physics formula for this
		//return AccelerationRate * 3.0f;
		return FMath::Clamp(acceleration_rate_ * 5.0f, acceleration_rate_ * 3.0f, acceleration_rate_ * 5.0f);
		//return 10000.0f;
	}

	bool Compare(FNPCVehicleInstance OtherInstance)
	{
		// only compare instanced mesh comp and instance index, other data may change over time!
		if (OtherInstance.instanced_mesh_comp_ == instanced_mesh_comp_
			&& OtherInstance.instance_index_ == instance_index_
			/*&& OtherInstance.DistAlongSpline == DistAlongSpline
			&& OtherInstance.ConnectingSplineIndex == ConnectingSplineIndex
			&& OtherInstance.LastRotation == LastRotation*/)
		{
			return true;
		}
		return false;
	}
};

// types of vehicles
// mesh, speed etc
// use this info to spawn vehicle instances
USTRUCT(BlueprintType)
struct FNPCVehicleType
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	UStaticMesh* static_mesh_;

	//float MaxSpeed;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	float car_length_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=VehicleType)
	float perc_chance_to_spawn_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	float max_speed_;

	UPROPERTY(EditAnywhere, Category=VehicleType)
	FVector spot_light_offset_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	UHierarchicalInstancedStaticMeshComponent* hism_;

	FNPCVehicleType()
	{
		max_speed_ = 1800.0f;
		car_length_ = 200.0f;
		perc_chance_to_spawn_ = 100.0f;
	}
};

class ABaseCarPath;

//class ACarPawn;

UCLASS()
class AIRSIM_API ANPCVehicleManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANPCVehicleManager();

	USceneComponent* root_comp_;

	// for debug only - shows how many vehicle instances we have
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Debug)
	int32 total_num_vehicle_instances_;

	// blueprint implementable event, override this with a proper get
	UFUNCTION(BlueprintCallable, Category=AirsimCar)
	TArray<AActor*> getAllAirsimCarPawns();

	UFUNCTION(BlueprintCallable, Category = Traffic)
	int32 getCurrentNumCarsPerPath();

	// spawning function. -1 for OverrideNumCarsPerPath sets it to spawn DefaultNumCarsPerPath
	UFUNCTION(BlueprintCallable, Category = CarSpawn)
	void initSpawnCarsOnPaths(int32 OverrideNumCarsPerPath = -1);

	USpotLightComponent* spawnCarLight();
	void removeCarLight(USpotLightComponent* spot_light_);

	// sets material collection to enable emissive on headlight mat, set light intensity to 13000.0f
	UFUNCTION(BlueprintCallable, Category=CarLight)
	void turnOnAllCarLights();

	// sets material collection to disable emissive on headlight mat, set light intensity to 0
	UFUNCTION(BlueprintCallable, Category = CarLight)
	void turnOffAllCarLights();


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AirsimCar)
	TArray<AActor*> local_pawns_;

	UPROPERTY(EditAnywhere, Category=Lights)
	bool use_car_headlights_;
protected:

	// location of the traffic UMaterialParameterCollection, params for car lights
	static const TCHAR* getTrafficParamsObjectPath()
	{
		return TEXT("/AirSim/Blueprints/Traffic/TrafficParams");
	}

	UFUNCTION(BlueprintCallable, Category=SimMode)
	ASimModeBase* getMySimModeBase();

	//UPROPERTY(BlueprintReadOnly, Category=SimMode)
	ASimModeBase* mySimModeBase_;

	// array of instanced meshes per vehicle mesh
	// perhaps this needs to be a struct? cause we may want other info, like %chance of this car etc
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleInfo)
	TArray<UHierarchicalInstancedStaticMeshComponent*> instanced_mesh_comps_;

	// all spotlights for all cars
	UPROPERTY(VisibleAnywhere, Category=Light)
	TArray<USpotLightComponent*> car_lights_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleInfo)
	TArray<FNPCVehicleType> vehicle_types_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleInfo)
	TArray<FNPCVehicleType> bicycle_types_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HISM)
	int32 HISM_cull_start_dist_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HISM)
	int32 HISM_cull_end_dist_;

	// destroy cars
	UFUNCTION(BlueprintCallable, Category = CarSpawn)
	void destroyAllCars();

	// UNUSED
	UHierarchicalInstancedStaticMeshComponent* GetRandomVehicleInstancedMeshComp();

	FNPCVehicleType GetRandomVehicleType();

	FNPCVehicleType getRandomBicycleType();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// DEFAULT how many cars to put on each path at beginning
	UPROPERTY(EditAnywhere,BlueprintReadOnly, Category=Spawning)
	int32 default_num_cars_per_path_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spawning)
	int32 default_num_bicycles_per_path_;

	UPROPERTY()
	int32 current_num_cars_per_path_;

	 // if true, only spawn on one car path for debugging
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Spawning)
	bool debug_only_one_car_path_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Spawning)
	int32 debug_max_cars_;

	UPROPERTY()
	bool has_complained_about_no_airsim_vehicles_;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
