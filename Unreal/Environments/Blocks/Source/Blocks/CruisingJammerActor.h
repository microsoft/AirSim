#pragma once

#include "CoreMinimal.h"
#include "JammerActor.h"
#include "CruisingJammerActor.generated.h"

/**
 * ACruisingJammerActor
 * 支持在固定区域内自动巡航的干扰器
 * 通过配置一组航路点来实现循环巡航
 */
UCLASS()
class BLOCKS_API ACruisingJammerActor : public AJammerActor
{
    GENERATED_BODY()

public:
    ACruisingJammerActor();

    virtual void Tick(float DeltaTime) override;
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;

protected:
    /** 巡航路径点（相对坐标），相对于 Actor 初始位置 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising")
    TArray<FVector> RelativeWaypoints;

    /** 移动 Actor 时是否保持航路点的世界坐标不变（自动调整相对坐标） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising")
    bool bKeepWorldWaypointsOnMove = true;

    /** 转向平滑度（0-1），值越大转向越平滑 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising", Meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float TurnSmoothness = 0.1f;

    /** 是否在开始时自动开始巡航 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising")
    bool bAutoStartCruising = true;

    /** 是否动态更新起始位置（在每次开始巡航时使用当前位置） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising")
    bool bDynamicStartLocation = true;

    /** 是否显示航路点调试信息 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising|Debug")
    bool bShowWaypoints = true;

    /** 航路点显示大小 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising|Debug")
    float WaypointSize = 50.f;

    /** 巡航模式 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising")
    bool bRelativeMode = true;  // 是否使用相对坐标模式

    /** 是否显示当前移动方向 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cruising|Debug")
    bool bShowDirection = true;

public:
    /** 开始巡航 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void StartCruising();

    /** 停止巡航 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void StopCruising();

    /** 更新起始位置到当前位置 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void UpdateStartLocation();

    /** 添加一个相对坐标航路点 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void AddRelativeWaypoint(const FVector& RelativeWaypoint);

    /** 添加一个世界坐标航路点（忽略相对坐标模式） */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void AddWorldWaypoint(const FVector& WorldWaypoint);

    /** 清除所有航路点 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    void ClearWaypoints();

    /** 获取当前航路点（世界坐标） */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    FVector GetCurrentWaypointLocation() const;

    /** 获取所有航路点（世界坐标） */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    TArray<FVector> GetAllWaypointLocations() const;

    /** 打印当前配置信息（用于调试） */
    UFUNCTION(BlueprintCallable, Category = "Cruising|Debug")
    void PrintDebugInfo();

    /** 获取当前航路点索引 */
    UFUNCTION(BlueprintCallable, Category = "Cruising")
    int32 GetCurrentWaypointIndex() const { return CurrentWaypointIndex; }

    /** 是否正在巡航 */
    bool IsCruising() const { return bIsCruising; }

protected:
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;

private:
    FRotator TargetRotation;

    void MoveToNextWaypoint();
    void DrawDebugInfo();
    bool HasValidWaypoints() const;
    FVector GetWaypointAt(int32 Index) const;
};
