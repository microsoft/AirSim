// Fill out your copyright notice in the Description page of Project Settings.
// checks and counts for violations

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SplinePathInfoLib.h"
#include "Components/BoxComponent.h"
#include "SplineViolationChecker.generated.h"

UCLASS()
class AIRSIM_API ASplineViolationChecker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASplineViolationChecker();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


	static const FSoftClassPath getViolationsMenuWidgetClass()
	{
		return FSoftClassPath(TEXT("UUserWidget'/AirSim/Blueprints/Traffic/UI/CarInfoWidget.CarInfoWidget_C'"));
	}

	// root component is necessary for attachment to vehicle pawns
	UPROPERTY()
	USceneComponent* default_scene_comp;

	UPROPERTY()
	UBaseViolationWidget* menu_widget;

public:	

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=Violations)
	TArray<FViolationCount> violation_counts_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Violations)
	TArray<FViolationInfo> violations_this_frame_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Violations)
	FSplinePathInfo spline_path_info_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Violations)
	float speed_;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Violations)
	float trace_size_ = 500.0f;

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// menus should start hidden. spline path info lib can call show menu for a vehicle actor
	UFUNCTION(BlueprintCallable, Category=Violation)
	void showMenu();

	UFUNCTION(BlueprintCallable, Category = Violation)
	void hideMenu();

	UPROPERTY(BlueprintReadOnly, Category=Violations)
	bool is_menu_visible_;

	//base sim mode will find all players and attach to all of them
	void setVehicleToAttachTo(AActor* parent);

	UPROPERTY()
	AActor* parent_vehicle_;
};
