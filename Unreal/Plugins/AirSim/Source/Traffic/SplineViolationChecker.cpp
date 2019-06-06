// Fill out your copyright notice in the Description page of Project Settings.

#include "SplineViolationChecker.h"
#include "BaseCarPath.h"
#include "BaseViolationWidget.h"


// Sets default values
ASplineViolationChecker::ASplineViolationChecker()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//Default violations with tolerances and minimum distances etc set here
	violation_counts_.Add(FViolationCount("Speed", EViolationType::VIOLATION_SPEED, 0.0f, 0, 0.0f, true, 0.0f));
	violation_counts_.Add(FViolationCount("Direction", EViolationType::VIOLATION_DIRECTION, 0.0f, 0.0, 0.25f, true, 100.0f)); // direction tolerance = 0.25
	violation_counts_.Add(FViolationCount("Lane", EViolationType::VIOLATION_LANE, 0.0f, 0, 200.0f, true, 0.0f)); //lane tolerance = 200.0f
	violation_counts_.Add(FViolationCount("Offroad", EViolationType::VIOLATION_OFFROAD, 0.0f, 0, 0.0f, true, -9999.0f));
	violation_counts_.Add(FViolationCount("RedLight", EViolationType::VIOLATION_REDLIGHT, 0.0f, 0, 0.0f, false, 0.0f));

	default_scene_comp = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneComp"));

	RootComponent = default_scene_comp;
}

// Called when the game starts or when spawned
void ASplineViolationChecker::BeginPlay()
{
	Super::BeginPlay();

	UWorld* World = GetWorld();

	UClass* MenuWidgetClass = getViolationsMenuWidgetClass().TryLoadClass<UUserWidget>();
	// construct menu here, but dont show it. we keep updating it during runtime
	if (MenuWidgetClass && World)
	{
		menu_widget = CreateWidget<UBaseViolationWidget>(World, MenuWidgetClass);
		if (menu_widget)
		{
			menu_widget->violation_checker = this;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, SplineViolationChecker could not spawn violation widget!"));

		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, SplineViolationChecker could not load violation widget!"));
	}
	
}

// Called every frame
void ASplineViolationChecker::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UWorld* World = GetWorld();
	if (parent_vehicle_ && World)
	{
		violation_counts_ = USplinePathInfoLib::updateViolationCountsForVehicle(World, parent_vehicle_, violation_counts_, trace_size_);		
		spline_path_info_ = USplinePathInfoLib::getCurrentSplineInfo(World, parent_vehicle_->GetActorLocation(), parent_vehicle_->GetActorRotation(), trace_size_);
		speed_ = USplinePathInfoLib::velocityToMPH(parent_vehicle_->GetVelocity());
	}

	//updateInfoWidget();

}
void ASplineViolationChecker::showMenu()
{
	if (menu_widget)
	{
		menu_widget->AddToViewport();
		is_menu_visible_ = true;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("MenuWidget is null"));
	}
}
void ASplineViolationChecker::hideMenu()
{
	if (menu_widget)
	{
		menu_widget->RemoveFromParent();
		is_menu_visible_ = false;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("MenuWidget is null"));
	}
}
void ASplineViolationChecker::setVehicleToAttachTo(AActor* parent)
{
	if (parent)
	{
		parent_vehicle_ = parent;
		AttachToActor(parent_vehicle_, FAttachmentTransformRules(EAttachmentRule::SnapToTarget, true));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("parent vehicle is null"));
	}
	
}

