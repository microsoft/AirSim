// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SkeletalMeshComponent.h"
#include "BasePedestrianPath.h"
#include "NPCPedestrianManager.generated.h"

USTRUCT(BlueprintType)
struct FPedestrianArchetype
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	USkeletalMesh* skel_mesh_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	UAnimationAsset* walking_anim_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	UAnimationAsset* idle_anim_;
};

USTRUCT(BlueprintType)
struct FPedestrianInfo
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SkeletalMeshCOmp)
	USkeletalMeshComponent* skel_mesh_comp_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VehicleType)
	UAnimationAsset* current_anim_;

	// current path
	UPROPERTY()
	ABasePedestrianPath* current_ped_path_;

	// next one we will move to, after reaching end of current ped path
	UPROPERTY()
	ABasePedestrianPath* next_ped_path_;


	// if we're moving from one path to another
	UPROPERTY()
	bool is_in_between_paths_;

	// last point we walked off from last path, interp from this point to next point on new ped path
	UPROPERTY()
	FVector last_ped_path_location_;

	// if true, traverse the splines in reverse
	UPROPERTY()
	bool is_spline_reverse_;

	UPROPERTY()
	float dist_on_spline_;

	UPROPERTY()
	float current_walk_speed_;


	UPROPERTY()
	float rotate_speed_;

	UPROPERTY()
	float max_walk_speed_;

	UPROPERTY()
	float spline_offset_x_;

	UPROPERTY()
	float spline_offset_y_;

	UPROPERTY()
	FPedestrianArchetype archetype_;

	FPedestrianInfo()
	{
		rotate_speed_ = 120.0f;
		is_in_between_paths_ = false;
		is_spline_reverse_ = false;
		dist_on_spline_ = 0.0f;
		current_walk_speed_ = 100.0f;
		last_ped_path_location_ = FVector(0, 0, 0);
		next_ped_path_ = NULL;
		current_ped_path_ = NULL;
		current_anim_ = NULL;
	}
};


UCLASS()
class AIRSIM_API ANPCPedestrianManager : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANPCPedestrianManager();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AirsimCar)
	TArray<AActor*> local_pawns_;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// blueprint implementable event, override this with a proper get
	UFUNCTION(BlueprintCallable, Category = AirsimCar)
	TArray<AActor*> getAllAirsimCarPawns();

	UPROPERTY(EditAnywhere, Category = Meshes)
	TArray<FPedestrianArchetype> pedestrian_archetypes_;

	void updatePedestrians(float DeltaTime);

	// spawned skel mesh comps
	UPROPERTY(VisibleAnywhere)
	TArray<FPedestrianInfo> pedestrians_;

	void PedestrianPlayAnim(int32 Index, UAnimationAsset* Animation, bool Loop);

	// keep track of ped paths that have collisions, so we can modify them when peds cross
	UPROPERTY(VisibleAnywhere)
	TArray<ABasePedestrianPath*> coll_enabled_ped_paths_;

	UPROPERTY()
	bool has_complained_about_no_airsim_vehicles_;

	// default max number of pedestrians spawned per spline
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Spawn)
	int32 default_num_pedestrians_per_spline_;

	UPROPERTY()
	int32 current_num_pedestrians_per_spline_;

	UPROPERTY(EditAnywhere, Category = Performance)
	float MaxDistCollCheck;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable, Category=Spawn)
	void spawnPedestrians(int32 OverrideNumPedestriansPerSpline = -1);

	UPROPERTY(EditAnywhere, Category = Spawn)
	bool debug_only_one_spline_ = false;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Spawn)
	int32 total_num_spawned_peds_;

	// if not set to -1, will try to limit spawned peds to this number
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=Spawn)
	int32 debug_max_peds_;

	UPROPERTY(EditAnywhere, Category = Speed)
	float walk_speed_min_;

	UPROPERTY(EditAnywhere, Category = Speed)
	float walk_speed_max_;

	UPROPERTY(EditAnywhere, Category = Offset)
	float rand_offset_min_;

	UPROPERTY(EditAnywhere, Category = Offset)
	float rand_offset_max_;

	UPROPERTY(EditAnywhere, Category = Perf)
	float max_draw_distance_;

	UPROPERTY(EditAnywhere, Category = Perf)
	bool is_collisions_enabled_;


	// CANNOT be renamed to airsim standards since they override UE4 default variables.
	// animation LODs for npcs pedestrian skeletal meshes.
	// for example 1.0 means screen size 1.0. index == number of frames to skip
	/* Array of MaxDistanceFactor to use for AnimUpdateRate when mesh is visible(rendered).
		MaxDistanceFactor is size on screen, as used by LODs
		Example :
	BaseVisibleDistanceFactorThesholds.Add(0.4f)
		BaseVisibleDistanceFactorThesholds.Add(0.2f)
		means :
		0 frame skip, MaxDistanceFactor > 0.4f
		1 frame skip, MaxDistanceFactor > 0.2f
		2 frame skip, MaxDistanceFactor > 0.0f
		*/
	UPROPERTY(EditAnywhere, Category = "Perf|Animation")
	TArray<float> BaseVisibleDistanceFactorThesholds;

	UPROPERTY(EditAnywhere, Category=Perf)
	bool bDisableSkelMeshTick = false;

	/** Rate of animation evaluation when non rendered (off screen and dedicated servers).
	* a value of 4 means evaluated 1 frame, then 3 frames skipped */
	UPROPERTY(EditAnywhere, Category = "Perf|Animation")
	float BaseNonRenderedUpdateRate;
	
	UFUNCTION(BlueprintCallable, Category=Pedestrian)
	int32 getCurrentNumPedsPerSpline();
};
