#pragma once

#include "CoreMinimal.h"
#include "DroneJammer.h"
#include "SectorJammer.generated.h"

/**
 * ASectorJammer
 * 扇形干扰器 - 基于 DroneJammer，提供扇形干扰区域和360度旋转功能
 */
UCLASS()
class BLOCKS_API ASectorJammer : public ADroneJammer
{
    GENERATED_BODY()

public:
    ASectorJammer();

    virtual void Tick(float DeltaTime) override;
    virtual void BeginPlay() override;

protected:
    /** 扇形干扰的半径（单位：cm） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Range")
    float SectorRadius = 3000.0f;

    /** 扇形的角度范围（单位：度，总角度） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Range", Meta = (ClampMin = "10.0", ClampMax = "360.0"))
    float SectorAngle = 60.0f;

    /** 扇形的当前朝向角度（单位：度，0度为正前方X轴） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Rotation")
    float SectorDirection = 0.0f;

    /** 是否启用自动旋转 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Rotation")
    bool bAutoRotate = false;

    /** 自动旋转速度（单位：度/秒） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Rotation")
    float RotationSpeed = 30.0f;

    /** 旋转方向（1为顺时针，-1为逆时针） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Rotation")
    int32 RotationDirection = 1;

    /** 是否显示扇形区域调试信息 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Debug")
    bool bShowSectorDebug = true;

    /** 扇形边界线的数量（用于调试显示） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Debug")
    int32 SectorDebugLines = 20;

    /** 中心球体的大小（半径，单位：cm） */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Debug", Meta = (ClampMin = "10.0"))
    float CenterSphereRadius = 100.0f;

    /** 扇形边界线的粗细 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SectorJammer|Debug", Meta = (ClampMin = "1.0", ClampMax = "50.0"))
    float SectorLineThickness = 10.0f;

public:
    /** 检查指定位置是否在扇形干扰范围内 */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    bool IsLocationInSector(const FVector& WorldLocation) const;

    /** 获取指定位置的干扰强度（考虑扇形范围） */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    float GetSectorJammerPowerAtLocation(const FVector& WorldLocation) const;

    /** 手动设置扇形朝向 */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    void SetSectorDirection(float NewDirection);

    /** 开始自动旋转 */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    void StartAutoRotation();

    /** 停止自动旋转 */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    void StopAutoRotation();

    /** 获取扇形的起始角度（世界坐标系） */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    float GetSectorStartAngle() const;

    /** 获取扇形的结束角度（世界坐标系） */
    UFUNCTION(BlueprintCallable, Category = "SectorJammer")
    float GetSectorEndAngle() const;

protected:
#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
    /** 绘制扇形调试信息 */
    void DrawSectorDebugInfo();

    /** 计算角度差（考虑360度循环） */
    float GetAngleDifference(float Angle1, float Angle2) const;

    /** 将角度标准化到 [0, 360) 范围 */
    float NormalizeAngle(float Angle) const;

    /** 将世界坐标转换为相对于Actor的角度 */
    float WorldLocationToAngle(const FVector& WorldLocation) const;
};
