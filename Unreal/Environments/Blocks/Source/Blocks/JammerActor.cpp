#include "JammerActor.h"
#include "Components/SphereComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"

AJammerActor::AJammerActor()
{
    PrimaryActorTick.bCanEverTick = true;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootComponent = Root;

    ShipMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("ShipMesh"));
    ShipMesh->SetupAttachment(RootComponent);

    JammerRange = CreateDefaultSubobject<USphereComponent>(TEXT("JammerRange"));
    JammerRange->SetupAttachment(RootComponent);
    JammerRange->InitSphereRadius(5000.f); // 50m（UE单位：cm）

    RangeVisualizer = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RangeVisualizer"));
    RangeVisualizer->SetupAttachment(RootComponent);
    RangeVisualizer->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    if (SphereMesh.Succeeded()) {
        RangeVisualizer->SetStaticMesh(SphereMesh.Object);
    }
}

void AJammerActor::BeginPlay()
{
    Super::BeginPlay();
    SyncVisualizerToRange();

    // 如果启用巡航，自动开始
    if (bEnableCruise)
    {
        StartCruise();
    }
}

void AJammerActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    SyncVisualizerToRange();
}

void AJammerActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 巡航逻辑
    if (bIsCruising && CruiseWaypoints.Num() > 0)
    {
        const FVector CurrentLocation = GetActorLocation();
        const FVector TargetWaypoint = GetCurrentTargetWaypoint();
        const FVector Direction = (TargetWaypoint - CurrentLocation).GetSafeNormal();
        const float Distance = FVector::Dist(CurrentLocation, TargetWaypoint);

        // 移动 Actor
        const FVector Movement = Direction * CruiseSpeed * DeltaTime;
        SetActorLocation(CurrentLocation + Movement, true);

        // 朝向移动方向
        if (Direction.SizeSquared() > 0.01f)
        {
            FRotator TargetRotation = FRotationMatrix::MakeFromX(Direction).Rotator();
            TargetRotation.Pitch = 0.f; // 保持水平
            TargetRotation.Roll = 0.f;
            SetActorRotation(TargetRotation);
        }

        // 检查是否到达目标点
        if (Distance <= ArrivalThreshold)
        {
            MoveToNextWaypoint();
        }

        // 绘制调试信息
        if (bShowCruisePath)
        {
            DrawCruiseDebugInfo();
        }
    }
}

void AJammerActor::SyncVisualizerToRange()
{
    if (!RangeVisualizer || !JammerRange) return;
    const float RadiusCm = JammerRange->GetUnscaledSphereRadius(); // 组件半径（cm）
    // 基础球体半径是 50cm，缩放系数 = 目标半径 / 50
    const float Scale = FMath::Max(0.01f, RadiusCm / 50.f);
    RangeVisualizer->SetRelativeScale3D(FVector(Scale));
    RangeVisualizer->SetVisibility(true);
    RangeVisualizer->SetRenderCustomDepth(true);
}

float AJammerActor::GetRadiusCm() const
{
    return JammerRange ? JammerRange->GetUnscaledSphereRadius() : 0.f;
}

float AJammerActor::ComputePowerAtLocation(const FVector& WorldLocation) const
{
    if (!bIsJamming) return 0.f;
    const FVector Src = GetActorLocation();
    const float DistCm = FVector::Distance(Src, WorldLocation);
    const float DistM = FMath::Max(0.1f, DistCm / 100.f); // 防止除零
    // 示例：1/r^2 衰减（你可替换为更真实的电磁模型/遮挡）
    return JammerPower / (DistM * DistM);
}

// ========== 巡航功能实现 ==========

void AJammerActor::StartCruise()
{
    if (!bEnableCruise) return;

    // 记录起始位置
    StartLocation = GetActorLocation();

    // 初始化巡航路径点
    InitializeCruiseWaypoints();

    if (CruiseWaypoints.Num() > 0)
    {
        bIsCruising = true;
        CurrentWaypointIndex = 0;
        UE_LOG(LogTemp, Warning, TEXT("[JammerActor] Started cruise with %d waypoints from location: %s"),
            CruiseWaypoints.Num(), *StartLocation.ToString());
    }
}

void AJammerActor::StopCruise()
{
    bIsCruising = false;
    UE_LOG(LogTemp, Warning, TEXT("[JammerActor] Cruise stopped"));
}

void AJammerActor::InitializeCruiseWaypoints()
{
    CruiseWaypoints.Empty();

    // 长方形巡航路径：
    // 1. 向左移动 2 单位 (CruiseWidth/2)
    // 2. 向上移动 1 单位 (CruiseHeight)
    // 3. 向右移动 4 单位 (CruiseWidth)
    // 4. 向下移动 1 单位 (CruiseHeight)
    // 5. 向左移动 2 单位 (CruiseWidth/2) 回到起点

    const float HalfWidth = CruiseWidth * 0.5f;

    // 起点
    CruiseWaypoints.Add(StartLocation);

    // 1. 向左移动 2 单位
    CruiseWaypoints.Add(StartLocation + FVector(0.f, -HalfWidth, 0.f));

    // 2. 向上移动 1 单位
    CruiseWaypoints.Add(StartLocation + FVector(CruiseHeight, -HalfWidth, 0.f));

    // 3. 向右移动 4 单位
    CruiseWaypoints.Add(StartLocation + FVector(CruiseHeight, HalfWidth, 0.f));

    // 4. 向下移动 1 单位
    CruiseWaypoints.Add(StartLocation + FVector(0.f, HalfWidth, 0.f));

    // 5. 回到起点（向左移动 2 单位）
    CruiseWaypoints.Add(StartLocation);

    UE_LOG(LogTemp, Log, TEXT("[JammerActor] Initialized %d cruise waypoints"), CruiseWaypoints.Num());
}

void AJammerActor::MoveToNextWaypoint()
{
    if (CruiseWaypoints.Num() == 0) return;

    CurrentWaypointIndex = (CurrentWaypointIndex + 1) % CruiseWaypoints.Num();

    UE_LOG(LogTemp, Log, TEXT("[JammerActor] Moving to waypoint %d: %s"),
        CurrentWaypointIndex, *GetCurrentTargetWaypoint().ToString());
}

FVector AJammerActor::GetCurrentTargetWaypoint() const
{
    if (CruiseWaypoints.Num() == 0 || CurrentWaypointIndex >= CruiseWaypoints.Num())
    {
        return GetActorLocation();
    }
    return CruiseWaypoints[CurrentWaypointIndex];
}

void AJammerActor::DrawCruiseDebugInfo()
{
    if (!GetWorld()) return;

    const FVector CurrentLocation = GetActorLocation();

    // 绘制所有航路点
    for (int32 i = 0; i < CruiseWaypoints.Num(); ++i)
    {
        const FLinearColor Color = (i == CurrentWaypointIndex) ? FLinearColor::Green : FLinearColor::Blue;

        // 绘制航路点球体
        DrawDebugSphere(GetWorld(), CruiseWaypoints[i], 25.f, 8, Color.ToFColor(true), false, -1.f, 0, 2.f);

        // 绘制连接线
        if (i < CruiseWaypoints.Num() - 1)
        {
            DrawDebugLine(GetWorld(), CruiseWaypoints[i], CruiseWaypoints[i + 1], FColor::Cyan, false, -1.f, 0, 3.f);
        }

        // 显示索引
        DrawDebugString(GetWorld(), CruiseWaypoints[i] + FVector(0.f, 0.f, 50.f),
            FString::Printf(TEXT("%d"), i), nullptr, FColor::White, -1.f);
    }

    // 绘制当前移动方向
    const FVector TargetWaypoint = GetCurrentTargetWaypoint();
    const FVector Direction = (TargetWaypoint - CurrentLocation).GetSafeNormal();
    DrawDebugLine(GetWorld(), CurrentLocation, CurrentLocation + Direction * 200.f,
        FColor::Red, false, -1.f, 0, 5.f);
}
