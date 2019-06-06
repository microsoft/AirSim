// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "TrafficControlInterface.h"
#include "AirSimSplineComp.h"
#include "BaseCarPathCollSphereComponent.h"
#include "NPCVehicleManager.h"

#include "Components/ArrowComponent.h"

#include "Components/SplineMeshComponent.h"

#include "Components/SpotLightComponent.h"

//#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

#include "BaseCarPath.generated.h"

//class USplineMeshComponent;

class ABaseIntersection;

USTRUCT(BlueprintType)
struct FConnectingPathStruct
{
	GENERATED_USTRUCT_BODY()

	// the path we connect to
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=CarPath)
	class ABaseCarPath* CarPath;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=ConnectingPath)
	UAirSimSplineComp* connecting_spline;

	// coll spheres for vehicles to check collision
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Collision)
	TArray<UBaseCarPathCollSphereComponent*> CollSpheres;

	// number of points we will have on the spline
	// on the MIDDLE of the spline, not including first and last
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	int32 NumSplineMidPoints = 1;
};

UCLASS()
class AIRSIM_API ABaseCarPath : public AActor, public ITrafficControlInterface
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
		next_traffic_status_ = ETrafficStatus::TRAFFIC_GREEN;
	}
	virtual void TrafficYellowLight_Implementation() override
	{
		next_traffic_status_ = ETrafficStatus::TRAFFIC_YELLOW;
	}
	virtual void TrafficRedLight_Implementation() override
	{
		next_traffic_status_ = ETrafficStatus::TRAFFIC_RED;
	}
	/** make this get whatever our current traffic light variable is */
	virtual ETrafficStatus GetTrafficLightStatus_Implementation() const override
	{
		return next_traffic_status_;
	}

	// Sets default values for this actor's properties
	ABaseCarPath();

	virtual void OnConstruction(const FTransform& Transform) override;

	UAirSimSplineComp* getCarPathSpline();

	// originally in the old API we had blueprints for these, now it's all c++
	TArray<UAirSimSplineComp*> GetEndConnectingSplines();

	UFUNCTION(BlueprintCallable, Category=Lane)
	int32 getLaneNumber() const;

	// spawns a vehicle on this path
	// NOTE vehicleindex is unused!!
	UFUNCTION(BlueprintCallable, Category = Lane)
	bool spawnVehicle(FNPCVehicleType VehicleType, int32 VehicleIndex);

	UFUNCTION(BlueprintCallable, Category = Lane)
	void destroyVehicles();

	// VehicleINdex is unused!!!
	void addVehicle(FNPCVehicleInstance VehicleInstance);

	bool removeVehicle(FNPCVehicleInstance VehicleInstance);

	// gets the last vehicle at the end of the path
	FNPCVehicleInstance GetLastVehicle();

	// checks for any collisions, OR predicted collisions (other paths that are enabled collision)
	// EnableCollSpheresOnNoOverlap makes the sphere colls collision enabled if no overlap
	// they'll get disabled again in DisableAllSphereColl, so they only last 1 tick
	UFUNCTION(BlueprintCallable, Category=Collision)
	bool checkConnectingPathCollision(int32 VehicleIndex, bool EnableCollSpheresOnNoOverlap = false);

	// set reserve path coll spheres to no coll
	//void DisableAllSphereColl();

	//checkConnectingPath
	UFUNCTION(BlueprintCallable, Category = Intersection)
	void setIntersection(ABaseIntersection* NewIntersection);

	UFUNCTION(BlueprintCallable, Category=Intersection)
	ABaseIntersection* getIntersection();


	// gets the latest vehicle waiting at the intersection about to move
	// checks if it has collisions on the connecting path it wants to take
	// if it has none, start moving and return true
	UFUNCTION(BlueprintCallable, Category=NextVehicle)
	bool nextWaitingVehicleStartMovingOnConnectingSpline();

	UPROPERTY()
	ANPCVehicleManager* MyVehicleManager;
	

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UFUNCTION(BlueprintCallable, CallInEditor, Category=CollSpheres)
	void generateConnectingSplineCollSpheres();

	//UFUNCTION(BlueprintCallable, CallInEditor, Category = ConnectingPaths)
	void cleanupOldSplines();

	// runs on construction
	// blueprint callaboe for debug only. TODO remove blueprint refernces
	UFUNCTION(BlueprintCallable, Category=Spline)
	void generateSplinesToConnectingPaths();

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "ConnectingPaths")
	bool manually_edit_spline_connector_points_;

	// TODO: fix by adding to struct
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ConnectingPaths", Meta = (MakeEditWidget = true))
	TArray<FTransform> spline_transform_widgets_;

	UPROPERTY()
	int32 spline_transform_widget_index;

	// sets the connecting paths splines and coll spheres
	// called in editor after setting connecting paths
	UFUNCTION(BlueprintCallable, CallInEditor, Category = ConnectingPaths)
	void regenConnectingSplines();

	// vehicle instances on this path
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = Vehicles)
	TArray<FNPCVehicleInstance> VehicleInstances;

	//UPROPERTY()
	//TArray<USpotLightComponent*> car_lights_;

	// move the vehicles forward along the spline, turn onto connecting splines if we reach the end
	void updateVehicleInstances(float DeltaSeconds);

	// how far are we from the next vehicle in the array?
	float GetDistanceToNextVehicle(int32 Index);

	// UNUSED
	float GetNextVehicleLength(int32 Index);

	// TODO: perhaps just get next vehicle function? 

	// traffic status of the next traffic volume
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=Traffic)
	ETrafficStatus next_traffic_status_;

	// move to cpp so it wont be garbage collected 
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category= SplineMeshComp)
	TArray <USplineMeshComponent*> SplineMeshComp;

	UPROPERTY(EditAnywhere, Category=Lane)
	int32 LaneNumber;

	UPROPERTY(VisibleDefaultsOnly, BlueprintReadWrite, Category=Spline)
	USceneComponent* RootComp;

	// possibly replace this with the follow comp's path, it looks better and is easier to customize
	// 5/21/2018: replacing with AirSimSplineComp
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=Spline)
	UAirSimSplineComp* CarPathSpline;

	// connecting paths at our end
	// manually added in editor
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=ConnectingPaths)
	TArray<FConnectingPathStruct> end_connecting_paths_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = SplineCollSphere)
	float SplineCollSphereSpacing;

	UPROPERTY(EditAnywhere,BlueprintReadOnly, Category= SplineCollSphere)
	float SplineCollSphereRadius;

	// for some reason we can't directly call the comp's generate spline meta data function
	// so we just do it here
	UFUNCTION(BlueprintCallable, Category=AirSimSplineComp)
	void generateSplineCompMetaData();

	// generate all spline meta data. by default, not all splines have metadata set up
	UFUNCTION(BlueprintCallable, CallInEditor, Category = MetaData)
	void generateAllSplineCompMetaData();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = MetaData)
	void applySpeedLimitToSelectedPoint();

	// applies speed limit to all spline points
	UFUNCTION(BlueprintCallable, CallInEditor, Category = MetaData)
	void applySpeedLimitToAll();

	UPROPERTY(EditAnywhere, Category = Metadata)
	float new_speed_limit_;

	// if true, at each spline point try to trace down and line us up to what we collide with
	// NOT IMPLEMENTED YET
	UPROPERTY(EditAnywhere, Category = Spline)
	bool bAlignSplinePointsZToRoad;

	// the intersection we end at
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=Intersection)
	ABaseIntersection* NextIntersection;

	UPROPERTY()
	TArray<UArrowComponent*> arrow_components_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Visualization)
	FVector arrow_size_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Visualization)
	float arrow_spacing_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Visualization)
	FColor arrow_color_;

	void makeArrowCompsAlongSpline();

	// meshes for collision checking along spline. hero car uses these for checking which spline im on
	void makeCollMeshesAlongSpline();


	// coll mesh stuff
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	UStaticMesh* coll_mesh_;

	UPROPERTY()
	TArray<USplineMeshComponent*> spline_meshes_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	float coll_mesh_scale_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	float coll_mesh_spacing_scale_;

	// it dynamically gets this using the spacing scale, mesh scale and the mesh bounds
	//UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	UPROPERTY()
	float coll_mesh_spacing_;

	UPROPERTY() //UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	float coll_mesh_tangent_length_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	float coll_mesh_end_tangent_length_;

	UPROPERTY()//UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	float coll_mesh_num_;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = CollMesh)
	bool use_spline_coll_mesh_;



public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
