// Fill out your copyright notice in the Description page of Project Settings.
// lib for checking spline for violation info

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AirSimSplineComp.h"
#include "BaseCarPath.h"
#include "BaseIntersection.h"
#include "SplinePathInfoLib.generated.h"

// TODO: this should be in an easier to find place
// this is the trace channel for checking splines to compare speeds and metadata with
// currently used for airsim car to compare its speed with the spline to check if over speed limit
#define COLLISION_TRAFFICTRACE		ECC_GameTraceChannel1

UENUM(BlueprintType)
enum class EViolationType : uint8
{
	VIOLATION_SPEED 	UMETA(DisplayName = "Speed"),
	VIOLATION_DIRECTION 	UMETA(DisplayName = "Direction"),
	VIOLATION_LANE	UMETA(DisplayName = "Lane"),
	VIOLATION_OFFROAD	UMETA(DisplayName = "Offroad"),
	VIOLATION_REDLIGHT	UMETA(DisplayName = "Red Light"),
	VIOLATION_MAX UMETA(DisplayName = "MAX UNUSED")
};
// this was coded in unreal standard not airsim sorry
// refactoring may be complicated, is currently used in a few blueprints and 
USTRUCT(BlueprintType)
struct FSplinePathInfo
{
	GENERATED_BODY()

	// spline metadata
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=spline_info)
	FAirSimSplinePointMetaData spline_meta_data_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	FRotator spline_direction_;

	// dist from closest point on lane
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	float dist_from_lane_;

	// next traffic light status
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	ETrafficStatus next_traffic_light_status_;

	// if no traffic light, -1
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	float next_traffic_light_dist_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	FVector next_traffic_light_loc_;

	// car path actor
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	ABaseCarPath* car_path_actor_;

	// spline comp
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	UAirSimSplineComp* spline_comp_;

	// all the spline comps we overlapped
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	TArray <UAirSimSplineComp*> all_spline_comps_;


	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = spline_info)
	bool is_in_intersection;

	// test making sure svn takes it
	FSplinePathInfo()
		: 
			/** Default constructor */
		spline_meta_data_(false, -1.0f),
		spline_direction_(FRotator(0.0f,0.0f,0.0f)),
		next_traffic_light_status_(ETrafficStatus::TRAFFIC_GREEN),
		car_path_actor_(nullptr),
		spline_comp_(nullptr),
		next_traffic_light_dist_(-1),
		next_traffic_light_loc_(FVector(0,0,0)),
		is_in_intersection(false)
	{}
};
///
USTRUCT(BlueprintType)
struct FViolationInfo
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	EViolationType violation_type_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	float current_violation_amount_;

	FViolationInfo()
	{
		violation_type_ = EViolationType::VIOLATION_SPEED;
		current_violation_amount_ = 0.0f;
	}

	FViolationInfo(EViolationType new_violation_type, float new_violation_amount)
	{
		violation_type_ = new_violation_type;
		current_violation_amount_ = new_violation_amount;
	}
};

// make it easier to keep track of violations
USTRUCT(BlueprintType)
struct FViolationCount
{
	GENERATED_BODY()

	// for printing to UI only, enum is used for sorting/counting
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	FString display_name_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	EViolationType violation_type_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	float current_violation_amount_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	int32 violation_count_;

	// if true, then ignore counting these violations in intersections
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	bool ignore_in_intersection_;

	// ignore if we are below this dist on current spline comp
	// useful for ignoring violations on first exiting an intersection
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	float min_dist_on_spline_;

	// default tolerance value. if violation amount is lower than tolerance, don't count as violation.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Violation)
	float tolerance_;

	FViolationCount()
	{
		current_violation_amount_ = 0.0f;
	}

	FViolationCount(FString name, EViolationType new_violation_type, float amount, int32 new_violation_count, float new_tolerance, bool set_ignore_in_intersection, float set_min_dist_on_spline)
	{
		display_name_ = name;
		violation_type_ = new_violation_type;
		current_violation_amount_ = amount;
		violation_count_ = new_violation_count;
		tolerance_ = new_tolerance;
		ignore_in_intersection_ = set_ignore_in_intersection;
		min_dist_on_spline_ = set_min_dist_on_spline;
	}
};

class ASplineViolationChecker;

//class ABaseTrafficGroupControlVolume;
/**
 * 
 */
UCLASS()
class AIRSIM_API USplinePathInfoLib : public UBlueprintFunctionLibrary//, public ITrafficControlInterface
{
	GENERATED_BODY()
public:

	// get spline meta data and other spline info in a struct
	// from our current location
	// trace size should be the size of the vehicle
	UFUNCTION(BlueprintCallable, Category = AirsimSpline)
	static FSplinePathInfo getCurrentSplineInfo(UWorld* world, FVector location, FRotator rotation, float trace_size);

	UFUNCTION(BlueprintCallable, Category=AirsimSpline)
	static float getDistOnSplineClosestToLocation(FVector Location, UAirSimSplineComp* spline_comp);

	// returns direction 100 is totally correct, -100 is totally wrong way
	UFUNCTION(BlueprintCallable, Category=AirsimSpline)
	static float getDirectionCheck(UAirSimSplineComp* spline_comp, FVector location, FRotator actor_rotation);

	// take in location, speed, spline info
	// return if we are speeding, facing wrong way or running red light
	// actor speed in MPH
	UFUNCTION(BlueprintCallable, Category=AirsimSpline)
	static TArray<FViolationInfo> checkForViolations(FSplinePathInfo spline_info, FVector actor_location, FRotator actor_rotation, float actor_speed);

	// convert velocity to MPH
	UFUNCTION(BlueprintCallable, Category=AirsimSpline)
	static float velocityToMPH(FVector Velocity);

	// adds violations to violation count
	// note, it returns a new violation count but does NOT modify the old one, since unreal cannot pass structs by ref
	static TArray<FViolationCount> addToViolationCounts(TArray<FViolationCount> violation_counts, TArray<FViolationInfo> violations, bool in_intersection, float current_dist_on_spline);

	// checks for violations at a vehicles location, returns violation count
	static TArray<FViolationCount> updateViolationCountsForVehicle(UWorld* world, AActor* vehicle, TArray<FViolationCount> current_violation_counts, float trace_size);

	// gets the violation counts from the vehicle's violation checker
	UFUNCTION(BlueprintCallable, Category = Violation)
	static TArray<FViolationCount> getVehicleCurrentViolations(AActor* vehicle);

	UFUNCTION(BlueprintCallable, Category = Violation)
	static ASplineViolationChecker* getAttachedViolationChecker(AActor* vehicle);

	UFUNCTION(BlueprintCallable, Category = Violation)
	static void showViolationMenu(AActor* vehicle);

	UFUNCTION(BlueprintCallable, Category = Violation)
	static void hideViolationMenu(AActor* vehicle);

	static void initActorsCheckViolation(UWorld* world, TArray<AActor*> local_pawns);
};
