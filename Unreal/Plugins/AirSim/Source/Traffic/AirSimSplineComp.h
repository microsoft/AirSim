// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SplineComponent.h"
#include "AirSimSplineComp.generated.h"

USTRUCT(BlueprintType)
struct FAirSimSplinePointMetaData
{
	GENERATED_BODY()

		// currently unused
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=AirsimSplinePointMetaData)
	bool is_school_zone = false;

	// speed limit in MPH
	// -1 is no speed limit
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=AirsimSplinePointMetaData)
	float speed_limit = 25.0f;

	/** Default constructor */
	FAirSimSplinePointMetaData()
		: is_school_zone(false), speed_limit(25.0f)
	{}

	/** Default constructor with values */
	FAirSimSplinePointMetaData(bool bSchoolZone, float Speed)
		: is_school_zone(bSchoolZone), speed_limit(Speed)
	{}
};
/**
 * 
 */
//UCLASS()
UCLASS(ClassGroup=Utility, ShowCategories = (Mobility), HideCategories = (Physics, Collision, Lighting, Rendering, Mobile), meta=(BlueprintSpawnableComponent))
class AIRSIM_API UAirSimSplineComp : public USplineComponent
{
	GENERATED_BODY()

public:
	// DO NOT REMOVE OR ADD!! will cause errors!
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=MetaData)
	TArray<FAirSimSplinePointMetaData> spline_metadata_;

	void insertMetaData(FAirSimSplinePointMetaData MetaData, int32 Index);

	UFUNCTION(BlueprintCallable, Category=AirSimSplineComp)
	void generateMetaData();

	//HACK have variables stored here, so visualizer and toolkit can share info
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Edit)
	int32 last_selected_index;

	void setNewLastSelectedIndex(int32 NewIndex);

};
