// Fill out your copyright notice in the Description page of Project Settings.


#include "AntennaJammer.h"
#include "Components/SphereComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SceneComponent.h"

// Sets default values
AAntennaJammer::AAntennaJammer()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// 根组件
    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootComponent = Root;

    // 雷达 SkeletalMesh
    ShipMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("ShipMesh"));
    ShipMesh->SetupAttachment(RootComponent);

    // 干扰范围球体
    JammerRange = CreateDefaultSubobject<USphereComponent>(TEXT("JammerRange"));
    JammerRange->SetupAttachment(RootComponent);
    JammerRange->InitSphereRadius(5000.f);

    // 新增：范围可视化组件
    RangeVisualizer = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RangeVisualizer"));
    RangeVisualizer->SetupAttachment(RootComponent);
    RangeVisualizer->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // 加载基础球体模型
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    if (SphereMesh.Succeeded()) {
        RangeVisualizer->SetStaticMesh(SphereMesh.Object);
        RangeVisualizer->SetRelativeScale3D(FVector(50.0f)); // 与半径对应缩放
    }

    // 默认参数
    JammerPower = 1.0f;
    bIsJamming = false;

}

// Called when the game starts or when spawned
void AAntennaJammer::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AAntennaJammer::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

