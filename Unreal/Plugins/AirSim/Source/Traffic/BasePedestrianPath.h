// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "TrafficControlInterface.h"
#include "BaseCarPathCollSphereComponent.h"

#include "Engine/StaticMesh.h"
#include "BasePedestrianPath.generated.h"

USTRUCT(BlueprintType)
struct FConnectingPedPathInfo
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PedPath)
	class ABasePedestrianPath* ConnectingPath;

	//TODO: blueprint check and collision for this!
	// if true, connects at spline index 0, otherwise connects at last index 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = PedPath)
	bool bConnectAt0;

	FConnectingPedPathInfo()
	{
		ConnectingPath = NULL;
		bConnectAt0 = false;
	}

	FConnectingPedPathInfo(ABasePedestrianPath* path, bool connect_at_zero)
	{
		ConnectingPath = path;
		bConnectAt0 = connect_at_zero;
	}
};

UCLASS()
class AIRSIM_API ABasePedestrianPath : public AActor, public ITrafficControlInterface
{
	GENERATED_BODY()
	
public:	

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficGreenLight();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficYellowLight();

	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		void TrafficRedLight();

	/** make this get whatever our current traffic light variable is */
	UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Traffic")
		ETrafficStatus GetTrafficLightStatus() const;


	virtual void TrafficGreenLight_Implementation() override
	{
		traffic_status_ = ETrafficStatus::TRAFFIC_GREEN;
	}
	virtual void TrafficYellowLight_Implementation() override
	{
		traffic_status_ = ETrafficStatus::TRAFFIC_YELLOW;
	}
	virtual void TrafficRedLight_Implementation() override
	{
		traffic_status_ = ETrafficStatus::TRAFFIC_RED;
	}
	/** make this get whatever our current traffic light variable is */
	virtual ETrafficStatus GetTrafficLightStatus_Implementation() const override
	{
		return traffic_status_;
	}


	// Sets default values for this actor's properties
	ABasePedestrianPath();

	virtual void OnConstruction(const FTransform& Transform) override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=NextPath)
	TArray<FConnectingPedPathInfo> NextPedPath;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = NextPath)
	TArray<FConnectingPedPathInfo> PrevPedPath;

	UFUNCTION(BlueprintCallable)
	FConnectingPedPathInfo GetRandomConnectingPath(bool bUseRear);

	//UFUNCTION(BlueprintCallable)
	bool GetIsWalkable();

	UPROPERTY(EditAnywhere)
	bool force_walking_disabled_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PedPath)
	bool bCanSpawnPedestrians;

	// crosswalks use these, they have coll enabled for cars so they wont run into peds
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = PedPath)
	bool bUseCollSpheres;

	//void disableAllCollSpheres();

	void enableCollSpheres(float Distance, bool bReverse);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	void disableEndSpheres();

	// delay before disabling end spheres, making sure that they are setup for the initial coll check
	FTimerHandle disable_end_spheres_timer;

	// used to get the next and last connecting paths
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = EndSphere)
	USphereComponent* start_sphere_;

	// used to get the next and last connecting paths
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = EndSphere)
	USphereComponent* end_sphere_;

	UPROPERTY(VisibleDefaultsOnly, BlueprintReadWrite, Category=Spline)
	class USplineComponent* spline_comp_; 

	UFUNCTION(BlueprintCallable, Category=Spline)
	void SetVisualizationWidth(float Width);

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category=Spline)
	float width_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = SplineCollSphere)
	float SplineCollSphereSpacing;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = SplineCollSphere)
	float SplineCollSphereRadius;

	// used for end and start spheres to detect nearby paths
	// that we should connect to
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = EndSphere)
	float end_sphere_coll_radius;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	ETrafficStatus traffic_status_;

	UPROPERTY()
	TArray<UStaticMeshComponent*> spline_meshes_;

	UPROPERTY()
	float spline_mesh_spacing_;

	UPROPERTY(EditDefaultsOnly, Category=Mesh)
	UStaticMesh* spline_mesh_;

	UPROPERTY()
	FVector spline_mesh_scale_;




public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	USplineComponent* getSplineComp();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Collision)
	TArray<UBaseCarPathCollSphereComponent*> CollSpheres;

	
	
};
