// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AntennaJammer.generated.h"

UCLASS()
class BLOCKS_API AAntennaJammer : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AAntennaJammer();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// 根组件（用于组织层级）
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    USceneComponent* Root;

	// 雷达模型（干扰源外观）
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    USkeletalMeshComponent* ShipMesh;

    /** 干扰范围球体（可用于检测或显示范围） */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer")
    class USphereComponent* JammerRange;

    /** 干扰强度 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer")
    float JammerPower;

    /** 是否开启干扰 */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Jammer")
    bool bIsJamming;

    // 用于显示干扰范围的球体
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jammer|Visual")
    UStaticMeshComponent* RangeVisualizer;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
