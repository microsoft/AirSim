#include "SectorJammer.h"
#include "Components/SphereComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

ASectorJammer::ASectorJammer()
{
    // 继承父类的基本设置
    PrimaryActorTick.bCanEverTick = true;

    // 设置默认的扇形参数
    SectorRadius = 3000.0f;  // 30米
    SectorAngle = 60.0f;     // 60度扇形
    SectorDirection = 0.0f;  // 朝向正前方
    bAutoRotate = false;
    RotationSpeed = 30.0f;   // 30度/秒
    RotationDirection = 1;   // 顺时针
    bShowSectorDebug = true;
    SectorDebugLines = 20;

    // 调整父类的球形范围以匹配扇形半径
    if (JammerRange)
    {
        JammerRange->SetSphereRadius(SectorRadius);
    }
}

void ASectorJammer::BeginPlay()
{
    Super::BeginPlay();

    // 同步球形组件的半径
    if (JammerRange)
    {
        JammerRange->SetSphereRadius(SectorRadius);
    }

    UE_LOG(LogTemp, Warning, TEXT("[SectorJammer] Started with Radius: %.1f, Angle: %.1f, Direction: %.1f"),
        SectorRadius, SectorAngle, SectorDirection);
}

void ASectorJammer::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 添加调试日志
    static int32 TickCounter = 0;
    if (++TickCounter % 60 == 0)  // 每秒打印一次
    {
        UE_LOG(LogTemp, Warning, TEXT("[SectorJammer] Tick #%d: bShowSectorDebug=%s, World=%s"),
            TickCounter, bShowSectorDebug ? TEXT("true") : TEXT("false"), GetWorld() ? TEXT("valid") : TEXT("null"));
    }

    // 自动旋转逻辑
    if (bAutoRotate)
    {
        float DeltaRotation = RotationSpeed * RotationDirection * DeltaTime;
        SectorDirection += DeltaRotation;
        SectorDirection = NormalizeAngle(SectorDirection);

        if (TickCounter % 60 == 0)
        {
            UE_LOG(LogTemp, Log, TEXT("[SectorJammer] Auto rotating: Direction=%.1f"), SectorDirection);
        }
    }

    // 绘制调试信息
    if (bShowSectorDebug && GetWorld())
    {
        DrawSectorDebugInfo();
    }
}

bool ASectorJammer::IsLocationInSector(const FVector& WorldLocation) const
{
    const FVector ActorLocation = GetActorLocation();
    const FVector ToTarget = WorldLocation - ActorLocation;
    const float Distance = ToTarget.Size();

    // 检查距离
    if (Distance > SectorRadius)
    {
        return false;
    }

    // 检查角度
    const float TargetAngle = WorldLocationToAngle(WorldLocation);
    const float HalfSectorAngle = SectorAngle * 0.5f;
    const float StartAngle = NormalizeAngle(SectorDirection - HalfSectorAngle);
    const float EndAngle = NormalizeAngle(SectorDirection + HalfSectorAngle);

    // 处理跨越0度的情况
    if (StartAngle > EndAngle)
    {
        return (TargetAngle >= StartAngle) || (TargetAngle <= EndAngle);
    }
    else
    {
        return (TargetAngle >= StartAngle) && (TargetAngle <= EndAngle);
    }
}

float ASectorJammer::GetSectorJammerPowerAtLocation(const FVector& WorldLocation) const
{
    if (!bIsJamming)
    {
        return 0.0f;
    }

    // 首先检查是否在扇形范围内
    if (!IsLocationInSector(WorldLocation))
    {
        return 0.0f;
    }

    // 计算基于距离的功率衰减
    const FVector ActorLocation = GetActorLocation();
    const float Distance = FVector::Dist(ActorLocation, WorldLocation);
    const float DistanceM = FMath::Max(0.1f, Distance / 100.0f); // 转换为米

    // 使用1/r²衰减模型
    return JammerPower / (DistanceM * DistanceM);
}

void ASectorJammer::SetSectorDirection(float NewDirection)
{
    SectorDirection = NormalizeAngle(NewDirection);
    UE_LOG(LogTemp, Log, TEXT("[SectorJammer] Direction set to: %.1f degrees"), SectorDirection);
}

void ASectorJammer::StartAutoRotation()
{
    bAutoRotate = true;
    UE_LOG(LogTemp, Warning, TEXT("[SectorJammer] Auto rotation started at %.1f deg/s"), RotationSpeed * RotationDirection);
}

void ASectorJammer::StopAutoRotation()
{
    bAutoRotate = false;
    UE_LOG(LogTemp, Warning, TEXT("[SectorJammer] Auto rotation stopped"));
}

float ASectorJammer::GetSectorStartAngle() const
{
    return NormalizeAngle(SectorDirection - SectorAngle * 0.5f);
}

float ASectorJammer::GetSectorEndAngle() const
{
    return NormalizeAngle(SectorDirection + SectorAngle * 0.5f);
}

#if WITH_EDITOR
void ASectorJammer::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    // 同步球形组件的半径
    if (JammerRange && PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(ASectorJammer, SectorRadius))
    {
        JammerRange->SetSphereRadius(SectorRadius);
    }

    // 标准化角度
    if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(ASectorJammer, SectorDirection))
    {
        SectorDirection = NormalizeAngle(SectorDirection);
    }
}
#endif

void ASectorJammer::DrawSectorDebugInfo()
{
    if (!GetWorld())
    {
        UE_LOG(LogTemp, Error, TEXT("[SectorJammer] DrawSectorDebugInfo: GetWorld() is null!"));
        return;
    }

    const FVector ActorLocation = GetActorLocation();
    const float HalfAngle = SectorAngle * 0.5f;
    const float StartAngleDeg = SectorDirection - HalfAngle;
    const float EndAngleDeg = SectorDirection + HalfAngle;

    // 添加调试日志
    static int32 DrawCounter = 0;
    if (++DrawCounter % 120 == 0)  // 每2秒打印一次
    {
        UE_LOG(LogTemp, Warning, TEXT("[SectorJammer] Drawing sector at %s, Radius=%.1f, Angle=%.1f, Direction=%.1f"),
            *ActorLocation.ToString(), SectorRadius, SectorAngle, SectorDirection);
    }

    // 绘制扇形边界线
    const FVector StartDirection = FVector(
        FMath::Cos(FMath::DegreesToRadians(StartAngleDeg)),
        FMath::Sin(FMath::DegreesToRadians(StartAngleDeg)),
        0.0f
    );
    const FVector EndDirection = FVector(
        FMath::Cos(FMath::DegreesToRadians(EndAngleDeg)),
        FMath::Sin(FMath::DegreesToRadians(EndAngleDeg)),
        0.0f
    );

    // 绘制扇形的两条边界线（使用可配置的线条粗细）
    DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + StartDirection * SectorRadius,
        FColor::Red, false, 0.0f, 0, SectorLineThickness);
    DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + EndDirection * SectorRadius,
        FColor::Red, false, 0.0f, 0, SectorLineThickness);

    // 绘制扇形弧线
    const float AngleStep = SectorAngle / SectorDebugLines;
    for (int32 i = 0; i < SectorDebugLines; ++i)
    {
        const float Angle1 = StartAngleDeg + i * AngleStep;
        const float Angle2 = StartAngleDeg + (i + 1) * AngleStep;

        const FVector Dir1 = FVector(
            FMath::Cos(FMath::DegreesToRadians(Angle1)),
            FMath::Sin(FMath::DegreesToRadians(Angle1)),
            0.0f
        );
        const FVector Dir2 = FVector(
            FMath::Cos(FMath::DegreesToRadians(Angle2)),
            FMath::Sin(FMath::DegreesToRadians(Angle2)),
            0.0f
        );

        DrawDebugLine(GetWorld(), ActorLocation + Dir1 * SectorRadius,
            ActorLocation + Dir2 * SectorRadius, FColor::Orange, false, 0.0f, 0, SectorLineThickness * 0.8f);
    }

    // 绘制中心指向线（当前朝向）
    const FVector CenterDirection = FVector(
        FMath::Cos(FMath::DegreesToRadians(SectorDirection)),
        FMath::Sin(FMath::DegreesToRadians(SectorDirection)),
        0.0f
    );
    DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + CenterDirection * SectorRadius,
        FColor::Green, false, 0.0f, 0, SectorLineThickness * 1.5f);

    // 绘制扇形填充区域（使用多条径向线）
    const int32 FillLines = FMath::Max(5, SectorDebugLines / 2);
    const float FillAngleStep = SectorAngle / FillLines;
    for (int32 i = 0; i <= FillLines; ++i)
    {
        const float FillAngle = StartAngleDeg + i * FillAngleStep;
        const FVector FillDir = FVector(
            FMath::Cos(FMath::DegreesToRadians(FillAngle)),
            FMath::Sin(FMath::DegreesToRadians(FillAngle)),
            0.0f
        );
        DrawDebugLine(GetWorld(), ActorLocation, ActorLocation + FillDir * SectorRadius,
            FColor::Yellow, false, 0.0f, 0, SectorLineThickness * 0.3f);
    }

    // 绘制中心球体来标记Actor位置（使用可配置的大小）
    DrawDebugSphere(GetWorld(), ActorLocation, CenterSphereRadius, 12, FColor::Magenta, false, 0.0f, 0, SectorLineThickness * 0.5f);

    // 显示信息文本
    const FString InfoText = FString::Printf(TEXT("Sector: %.1f° @ %.1f°\nRadius: %.0fcm\nAuto: %s"),
        SectorAngle, SectorDirection, SectorRadius, bAutoRotate ? TEXT("ON") : TEXT("OFF"));
    DrawDebugString(GetWorld(), ActorLocation + FVector(0.f, 0.f, 300.f),
        InfoText, nullptr, FColor::White, 0.0f, true);
}

float ASectorJammer::GetAngleDifference(float Angle1, float Angle2) const
{
    float Diff = FMath::Abs(Angle1 - Angle2);
    if (Diff > 180.0f)
    {
        Diff = 360.0f - Diff;
    }
    return Diff;
}

float ASectorJammer::NormalizeAngle(float Angle) const
{
    while (Angle < 0.0f)
    {
        Angle += 360.0f;
    }
    while (Angle >= 360.0f)
    {
        Angle -= 360.0f;
    }
    return Angle;
}

float ASectorJammer::WorldLocationToAngle(const FVector& WorldLocation) const
{
    const FVector ActorLocation = GetActorLocation();
    const FVector ToTarget = WorldLocation - ActorLocation;

    // 计算相对于X轴的角度（UE坐标系）
    float Angle = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
    return NormalizeAngle(Angle);
}
