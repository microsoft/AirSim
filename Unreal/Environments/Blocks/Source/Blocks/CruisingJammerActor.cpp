#include "CruisingJammerActor.h"
#include "DrawDebugHelpers.h"

ACruisingJammerActor::ACruisingJammerActor()
{
    // 保持父类的 Tick 启用
    // 注意：在构造函数中 GetActorLocation() 可能不可靠，所以在 OnConstruction 中初始化
}

void ACruisingJammerActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    const FVector CurrentLocation = GetActorLocation();

    // 如果 StartLocation 还没有初始化（第一次放置）
    if (StartLocation.IsNearlyZero())
    {
        StartLocation = CurrentLocation;
        UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] OnConstruction: First time initialization, StartLocation = %s"), *StartLocation.ToString());
    }
    // 如果 Actor 被移动了
    else if (!StartLocation.Equals(CurrentLocation, 1.0f))
    {
        const FVector LocationDelta = CurrentLocation - StartLocation;

        if (bKeepWorldWaypointsOnMove && RelativeWaypoints.Num() > 0)
        {
            // 保持世界坐标不变，调整相对坐标
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] OnConstruction: Actor moved from %s to %s (delta: %s)"),
                *StartLocation.ToString(), *CurrentLocation.ToString(), *LocationDelta.ToString());

            // 调整相对坐标：新相对坐标 = 旧相对坐标 - 位置变化
            for (int32 i = 0; i < RelativeWaypoints.Num(); ++i)
            {
                FVector OldRelative = RelativeWaypoints[i];
                RelativeWaypoints[i] = RelativeWaypoints[i] - LocationDelta;
                UE_LOG(LogTemp, Log, TEXT("[CruisingJammer] Adjusted Waypoint[%d]: %s -> %s"),
                    i, *OldRelative.ToString(), *RelativeWaypoints[i].ToString());
            }

            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] Adjusted %d waypoints to keep world coordinates"), RelativeWaypoints.Num());
        }
        else if (bDynamicStartLocation)
        {
            // 动态模式：更新起始位置，相对坐标不变
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] OnConstruction: Dynamic mode, updating StartLocation from %s to %s"),
                *StartLocation.ToString(), *CurrentLocation.ToString());
        }

        StartLocation = CurrentLocation;
    }

    // 在编辑器中绘制预览
    if (GetWorld() && bShowWaypoints && RelativeWaypoints.Num() > 0)
    {
        FlushPersistentDebugLines(GetWorld());

        const TArray<FVector> WaypointLocations = GetAllWaypointLocations();

        UE_LOG(LogTemp, Log, TEXT("[CruisingJammer] Editor Preview: StartLocation=%s, Drawing %d waypoints"),
            *StartLocation.ToString(), WaypointLocations.Num());

        for (int32 i = 0; i < WaypointLocations.Num(); ++i)
        {
            DrawDebugSphere(GetWorld(), WaypointLocations[i], WaypointSize * 0.5f, 12, FColor::Cyan, true, -1.f, 0, 2.f);

            if (i < WaypointLocations.Num() - 1)
            {
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[i + 1], FColor::Cyan, true, -1.f, 0, 2.f);
            }
            else
            {
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[0], FColor::Cyan, true, -1.f, 0, 2.f);
            }

            DrawDebugString(GetWorld(), WaypointLocations[i] + FVector(0.f, 0.f, WaypointSize),
                FString::Printf(TEXT("%d"), i), nullptr, FColor::White, -1.f);
        }

        // 显示起始位置
        DrawDebugSphere(GetWorld(), StartLocation, 100.f, 12, FColor::Red, true, -1.f, 0, 3.f);
        DrawDebugString(GetWorld(), StartLocation + FVector(0.f, 0.f, 200.f),
            TEXT("START"), nullptr, FColor::Red, -1.f);
    }
}

void ACruisingJammerActor::BeginPlay()
{
    Super::BeginPlay();

    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] ===== BeginPlay START ====="));
    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] Actor Location: %s"), *GetActorLocation().ToString());
    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] StartLocation (before): %s"), *StartLocation.ToString());

    // 确保 StartLocation 被初始化
    if (StartLocation.IsNearlyZero() || bDynamicStartLocation)
    {
        StartLocation = GetActorLocation();
        UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] StartLocation updated to: %s"), *StartLocation.ToString());
    }

    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] bAutoStartCruising: %s"), bAutoStartCruising ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] HasValidWaypoints: %s"), HasValidWaypoints() ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] RelativeWaypoints.Num: %d"), RelativeWaypoints.Num());

    if (bAutoStartCruising && HasValidWaypoints())
    {
        UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] Auto-starting cruise..."));
        StartCruising();
    }
    else
    {
        if (!bAutoStartCruising)
            UE_LOG(LogTemp, Error, TEXT("[CruisingJammer] NOT starting cruise: bAutoStartCruising is false"));
        if (!HasValidWaypoints())
            UE_LOG(LogTemp, Error, TEXT("[CruisingJammer] NOT starting cruise: Need at least 2 waypoints, have %d"), RelativeWaypoints.Num());
    }

    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] ===== BeginPlay END ====="));
}

void ACruisingJammerActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bIsCruising && HasValidWaypoints())
    {
        const FVector CurrentLocation = GetActorLocation();

        // 【关键修复】检测世界平移，自动更新 StartLocation
        static FVector LastKnownActorLocation = FVector::ZeroVector;
        if (LastKnownActorLocation.IsZero())
        {
            LastKnownActorLocation = CurrentLocation;
        }

        // 如果船的位置发生了大幅跳跃（可能是世界平移），更新 StartLocation
        const float LocationJump = FVector::Dist(LastKnownActorLocation, CurrentLocation);
        if (LocationJump > 10000.0f) // 100米以上的跳跃认为是世界平移
        {
            FVector LocationDelta = CurrentLocation - LastKnownActorLocation;
            StartLocation += LocationDelta;
            UE_LOG(LogTemp, Error, TEXT("[CruisingJammer] WORLD TRANSLATION DETECTED! Jump: %.2f cm"), LocationJump);
            UE_LOG(LogTemp, Error, TEXT("  StartLocation updated by delta: %s"), *LocationDelta.ToString());
            UE_LOG(LogTemp, Error, TEXT("  New StartLocation: %s"), *StartLocation.ToString());
        }
        LastKnownActorLocation = CurrentLocation;

        const FVector TargetWaypoint = GetWaypointAt(CurrentWaypointIndex);
        const FVector Direction = (TargetWaypoint - CurrentLocation).GetSafeNormal();
        const float Distance = FVector::Dist(CurrentLocation, TargetWaypoint);

        // 详细的 Tick 日志（每 60 帧打印一次，避免刷屏）
        static int32 TickCounter = 0;
        if (++TickCounter % 60 == 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] TICK #%d:"), TickCounter);
            UE_LOG(LogTemp, Warning, TEXT("  Current Actor Location: %s"), *CurrentLocation.ToString());
            UE_LOG(LogTemp, Warning, TEXT("  Target Waypoint[%d]: %s"), CurrentWaypointIndex, *TargetWaypoint.ToString());
            UE_LOG(LogTemp, Warning, TEXT("  Distance to target: %.2f cm"), Distance);
            UE_LOG(LogTemp, Warning, TEXT("  Movement Direction: %s"), *Direction.ToString());
            UE_LOG(LogTemp, Warning, TEXT("  Movement Speed: %.2f cm/s"), CruiseSpeed);
        }

        // 移动 Actor
        const FVector Movement = Direction * CruiseSpeed * DeltaTime;
        SetActorLocation(CurrentLocation + Movement, true);

        // 计算目标旋转（朝向移动方向）
        if (Direction.SizeSquared() > 0.01f)
        {
            TargetRotation = FRotationMatrix::MakeFromX(Direction).Rotator();
        }

        // 平滑转向
        if (TurnSmoothness > 0.f)
        {
            const FRotator CurrentRotation = GetActorRotation();
            FRotator NewRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, (1.f - TurnSmoothness) * 10.f);
            NewRotation.Pitch = 0.f; // 保持水平，不俯仰
            NewRotation.Roll = 0.f;  // 不侧倾
            SetActorRotation(NewRotation);
        }
        else
        {
            FRotator NewRotation = TargetRotation;
            NewRotation.Pitch = 0.f;
            NewRotation.Roll = 0.f;
            SetActorRotation(NewRotation);
        }

        // 检查是否到达目标点
        if (Distance <= ArrivalThreshold)
        {
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] ARRIVED at Waypoint[%d]! Distance: %.2f <= %.2f"),
                CurrentWaypointIndex, Distance, ArrivalThreshold);
            MoveToNextWaypoint();
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] Next target: Waypoint[%d] at %s"),
                CurrentWaypointIndex, *GetWaypointAt(CurrentWaypointIndex).ToString());
        }

        // 绘制调试信息
        if (bShowWaypoints || bShowDirection)
        {
            DrawDebugInfo();
        }
    }
}

void ACruisingJammerActor::StartCruising()
{
    if (HasValidWaypoints())
    {
        // 【关键修复】强制更新 StartLocation 为船的当前位置
        FVector OldStartLocation = StartLocation;
        StartLocation = GetActorLocation();

        UE_LOG(LogTemp, Error, TEXT("[CruisingJammer] !!!!! StartCruising CRITICAL DEBUG !!!!!"));
        UE_LOG(LogTemp, Error, TEXT("  Old StartLocation: %s"), *OldStartLocation.ToString());
        UE_LOG(LogTemp, Error, TEXT("  New StartLocation (Actor Location): %s"), *StartLocation.ToString());
        UE_LOG(LogTemp, Error, TEXT("  bDynamicStartLocation: %s"), bDynamicStartLocation ? TEXT("true") : TEXT("false"));

        bIsCruising = true;

        // 找到最近的航路点作为起点
        float MinDistance = TNumericLimits<float>::Max();
        const TArray<FVector> WaypointLocations = GetAllWaypointLocations();

        UE_LOG(LogTemp, Error, TEXT("  Recalculated World Waypoints:"));
        for (int32 i = 0; i < WaypointLocations.Num(); ++i)
        {
            const float Distance = FVector::Dist(GetActorLocation(), WaypointLocations[i]);
            UE_LOG(LogTemp, Error, TEXT("    [%d]: %s (Distance: %.2f cm)"), i, *WaypointLocations[i].ToString(), Distance);

            if (Distance < MinDistance)
            {
                MinDistance = Distance;
                CurrentWaypointIndex = i;
            }
        }

        UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] Starting cruise from waypoint %d (distance: %.2f cm)"), CurrentWaypointIndex, MinDistance);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[CruisingJammer] Cannot start cruising: Need at least 2 waypoints, have %d"), RelativeWaypoints.Num());
    }
}

void ACruisingJammerActor::StopCruising()
{
    bIsCruising = false;
}

void ACruisingJammerActor::UpdateStartLocation()
{
    FVector OldLocation = StartLocation;
    StartLocation = GetActorLocation();
    UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] UpdateStartLocation: Changed from %s to %s"), *OldLocation.ToString(), *StartLocation.ToString());
}

void ACruisingJammerActor::AddRelativeWaypoint(const FVector& RelativeWaypoint)
{
    RelativeWaypoints.Add(RelativeWaypoint);
}

void ACruisingJammerActor::AddWorldWaypoint(const FVector& WorldWaypoint)
{
    RelativeWaypoints.Add(WorldWaypoint - StartLocation);
}

void ACruisingJammerActor::ClearWaypoints()
{
    RelativeWaypoints.Empty();
    bIsCruising = false;
    CurrentWaypointIndex = 0;
}

void ACruisingJammerActor::MoveToNextWaypoint()
{
    if (RelativeWaypoints.Num() == 0) return;

    CurrentWaypointIndex = (CurrentWaypointIndex + 1) % RelativeWaypoints.Num();
}

void ACruisingJammerActor::DrawDebugInfo()
{
    const FVector CurrentLocation = GetActorLocation();

    // 显示航路点
    if (bShowWaypoints)
    {
        const TArray<FVector> WaypointLocations = GetAllWaypointLocations();

        // 每 120 帧打印一次调试球体位置（避免刷屏）
        static int32 DrawCounter = 0;
        if (++DrawCounter % 120 == 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("[CruisingJammer] ===== DEBUG SPHERES POSITIONS ====="));
            UE_LOG(LogTemp, Warning, TEXT("  Ship Location: %s"), *CurrentLocation.ToString());
        }

        for (int32 i = 0; i < WaypointLocations.Num(); ++i)
        {
            const FLinearColor Color = (i == CurrentWaypointIndex) ? FLinearColor::Green : FLinearColor::Blue;
            const FString ColorName = (i == CurrentWaypointIndex) ? TEXT("GREEN") : TEXT("BLUE");

            // 绘制航路点球体
            DrawDebugSphere(GetWorld(), WaypointLocations[i], WaypointSize * 0.5f, 12, Color.ToFColor(true), false);

            // 每 120 帧打印一次球体位置
            if (DrawCounter % 120 == 0)
            {
                const float Distance = FVector::Dist(CurrentLocation, WaypointLocations[i]);
                UE_LOG(LogTemp, Warning, TEXT("  %s Sphere[%d]: %s (Distance: %.2f cm)"),
                    *ColorName, i, *WaypointLocations[i].ToString(), Distance);
            }

            // 绘制连接线
            if (i < WaypointLocations.Num() - 1)
            {
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[i + 1], FColor::Blue, false, -1.f, 0, 5.f);
            }
            else
            {
                // 闭合路径（从最后一点连回第一点）
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[0], FColor::Blue, false, -1.f, 0, 5.f);
            }

            // 显示索引
            DrawDebugString(GetWorld(), WaypointLocations[i] + FVector(0.f, 0.f, WaypointSize),
                FString::Printf(TEXT("%d"), i), nullptr, FColor::White);
        }

        if (DrawCounter % 120 == 0)
        {
            UE_LOG(LogTemp, Warning, TEXT("============================================="));
        }
    }

    // 显示移动方向
    if (bShowDirection && HasValidWaypoints())
    {
        const FVector TargetWaypoint = GetWaypointAt(CurrentWaypointIndex);
        const FVector Direction = (TargetWaypoint - CurrentLocation).GetSafeNormal();
        DrawDebugLine(GetWorld(), CurrentLocation, CurrentLocation + Direction * 500.f,
            FColor::Red, false, -1.f, 0, 5.f);
    }
}

bool ACruisingJammerActor::HasValidWaypoints() const
{
    return RelativeWaypoints.Num() >= 2; // 至少需要2个点才能巡航
}

FVector ACruisingJammerActor::GetCurrentWaypointLocation() const
{
    return GetWaypointAt(CurrentWaypointIndex);
}

TArray<FVector> ACruisingJammerActor::GetAllWaypointLocations() const
{
    TArray<FVector> WorldWaypoints;
    for (const FVector& RelativePoint : RelativeWaypoints)
    {
        WorldWaypoints.Add(StartLocation + RelativePoint);
    }
    return WorldWaypoints;
}

FVector ACruisingJammerActor::GetWaypointAt(int32 Index) const
{
    if (Index >= 0 && Index < RelativeWaypoints.Num())
    {
        return StartLocation + RelativeWaypoints[Index];
    }
    return FVector::ZeroVector;
}

void ACruisingJammerActor::PrintDebugInfo()
{
    UE_LOG(LogTemp, Warning, TEXT("========== CruisingJammerActor Debug Info =========="));
    UE_LOG(LogTemp, Warning, TEXT("Actor Location: %s"), *GetActorLocation().ToString());
    UE_LOG(LogTemp, Warning, TEXT("StartLocation: %s"), *StartLocation.ToString());
    UE_LOG(LogTemp, Warning, TEXT("bDynamicStartLocation: %s"), bDynamicStartLocation ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("bKeepWorldWaypointsOnMove: %s"), bKeepWorldWaypointsOnMove ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("bIsCruising: %s"), bIsCruising ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Warning, TEXT("CurrentWaypointIndex: %d"), CurrentWaypointIndex);
    UE_LOG(LogTemp, Warning, TEXT("RelativeWaypoints Count: %d"), RelativeWaypoints.Num());

    UE_LOG(LogTemp, Warning, TEXT("--- Relative Waypoints ---"));
    for (int32 i = 0; i < RelativeWaypoints.Num(); ++i)
    {
        UE_LOG(LogTemp, Warning, TEXT("  [%d] Relative: %s"), i, *RelativeWaypoints[i].ToString());
    }

    UE_LOG(LogTemp, Warning, TEXT("--- World Waypoints ---"));
    const TArray<FVector> WorldWaypoints = GetAllWaypointLocations();
    for (int32 i = 0; i < WorldWaypoints.Num(); ++i)
    {
        const float Distance = FVector::Dist(GetActorLocation(), WorldWaypoints[i]);
        const FString Marker = (i == CurrentWaypointIndex) ? TEXT(" <- CURRENT TARGET") : TEXT("");
        UE_LOG(LogTemp, Warning, TEXT("  [%d] World: %s (Distance: %.2f cm)%s"),
            i, *WorldWaypoints[i].ToString(), Distance, *Marker);
    }

    UE_LOG(LogTemp, Warning, TEXT("==================================================="));
}

void ACruisingJammerActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    // 在编辑器中修改属性时，确保 StartLocation 是最新的
    if (bDynamicStartLocation)
    {
        StartLocation = GetActorLocation();
        UE_LOG(LogTemp, Log, TEXT("[CruisingJammer] PostEditChangeProperty: StartLocation updated to %s"), *StartLocation.ToString());
    }

    // 编辑器中预览巡航路径
    if (GetWorld() && bShowWaypoints && RelativeWaypoints.Num() > 0)
    {
        // 清除旧的调试绘制，重新绘制
        FlushPersistentDebugLines(GetWorld());

        // 在编辑器中绘制航路点预览
        const TArray<FVector> WaypointLocations = GetAllWaypointLocations();
        for (int32 i = 0; i < WaypointLocations.Num(); ++i)
        {
            // 绘制持久化的调试球体
            DrawDebugSphere(GetWorld(), WaypointLocations[i], WaypointSize * 0.5f, 12, FColor::Cyan, true, -1.f, 0, 2.f);

            // 绘制连接线
            if (i < WaypointLocations.Num() - 1)
            {
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[i + 1], FColor::Cyan, true, -1.f, 0, 2.f);
            }
            else
            {
                // 闭合路径
                DrawDebugLine(GetWorld(), WaypointLocations[i], WaypointLocations[0], FColor::Cyan, true, -1.f, 0, 2.f);
            }

            // 显示索引
            DrawDebugString(GetWorld(), WaypointLocations[i] + FVector(0.f, 0.f, WaypointSize),
                FString::Printf(TEXT("%d"), i), nullptr, FColor::White, -1.f);
        }

        // 显示起始位置
        DrawDebugSphere(GetWorld(), StartLocation, 100.f, 12, FColor::Red, true, -1.f, 0, 3.f);
        DrawDebugString(GetWorld(), StartLocation + FVector(0.f, 0.f, 200.f),
            TEXT("START"), nullptr, FColor::Red, -1.f);

        UE_LOG(LogTemp, Log, TEXT("[CruisingJammer] Editor Preview: StartLocation=%s, Waypoints=%d"),
            *StartLocation.ToString(), WaypointLocations.Num());
    }
}
