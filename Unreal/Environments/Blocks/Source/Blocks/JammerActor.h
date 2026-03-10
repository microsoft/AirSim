#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "JammerActor.generated.h"

class USphereComponent;
class USkeletalMeshComponent;
class UStaticMeshComponent;

UCLASS()
class BLOCKS_API AJammerActor : public AActor
{
    GENERATED_BODY()
public:
    AJammerActor();
    virtual void Tick(float DeltaTime) override;
    virtual void OnConstruction(const FTransform& Transform) override;

protected:
    virtual void BeginPlay() override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    USceneComponent* Root;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    USkeletalMeshComponent* ShipMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    USphereComponent* JammerRange;

    /** ����׼���ʡ����书�ʣ����ⵥλ�������ģ��һ�¼��ɣ� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Params")
    float JammerPower = 1.0f;

    /** �Ƿ������� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Params")
    bool bIsJamming = true;

    /** �Ƿ�����Ѳ��ģʽ */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise")
    bool bEnableCruise = false;

    /** Ѳ���ٶȣ���λ��cm/s�� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise")
    float CruiseSpeed = 500.0f;

    /** �����Ѳ��Ŀ��ȣ���λ��cm�����������ƶ��ľ��룩 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise")
    float CruiseWidth = 400.0f;  // 4��λ = 400cm

    /** �����Ѳ��ĸ߶ȣ���λ��cm�����������ƶ��ľ��룩 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise")
    float CruiseHeight = 100.0f;  // 1��λ = 100cm

    /** �ﵽĿ���ľ����ֵ����λ��cm�� */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise")
    float ArrivalThreshold = 50.0f;

    /** �Ƿ���ʾѲ��·������Ϣ */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer|Cruise|Debug")
    bool bShowCruisePath = true;

    /** ���ӻ���Χ���壨����ʾ�� */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer|Visual")
    UStaticMeshComponent* RangeVisualizer;

public:
    /** ����ǰ���ʣ���������˥���� */
    UFUNCTION(BlueprintCallable, Category = "Jammer")
    float GetBasePower() const { return JammerPower; }

    /** ���Ƿ����ڸ��� */
    UFUNCTION(BlueprintCallable, Category = "Jammer")
    bool IsJamming() const { return bIsJamming; }

    /** �뾶���� JammerRange �İ뾶Ϊ׼����λ cm�� */
    UFUNCTION(BlueprintCallable, Category = "Jammer")
    float GetRadiusCm() const;

    /** ��ѡ����λ�ü��㡰λ����ع��ʡ��������ɿռ�˥��ʾ�������滻Ϊ���ģ�ͣ� */
    UFUNCTION(BlueprintCallable, Category = "Jammer")
    float ComputePowerAtLocation(const FVector& WorldLocation) const;

    // JammerActor.h
    UFUNCTION(BlueprintCallable, Category = "Jammer")
    float GetJammerPower() const { return bIsJamming ? JammerPower : 0.f; }

    UFUNCTION(BlueprintCallable, Category = "Jammer")
    float GetJammerPowerAtLocation(const FVector& WorldLocation) const
    {
        if (!bIsJamming) return 0.f;
        const float distM = FMath::Max(0.1f, FVector::Distance(GetActorLocation(), WorldLocation) / 100.f);
        return JammerPower / (distM * distM); // ʾ����1/r?
    }

    /** ��ʼѲ�� */
    UFUNCTION(BlueprintCallable, Category = "Jammer|Cruise")
    void StartCruise();

    /** ֹͣѲ�� */
    UFUNCTION(BlueprintCallable, Category = "Jammer|Cruise")
    void StopCruise();

    /** ��ȡ��ǰ�Ƿ���Ѳ�� */
    UFUNCTION(BlueprintCallable, Category = "Jammer|Cruise")
    bool IsCruising() const { return bIsCruising; }

protected:
    // 巡航相关的成员变量（供派生类访问）
    bool bIsCruising = false;
    FVector StartLocation = FVector::ZeroVector;
    int32 CurrentWaypointIndex = 0;
    TArray<FVector> CruiseWaypoints;

private:
    void SyncVisualizerToRange();

    // 巡航相关的私有函数
    void InitializeCruiseWaypoints();
    void MoveToNextWaypoint();
    void DrawCruiseDebugInfo();
    FVector GetCurrentTargetWaypoint() const;
};
