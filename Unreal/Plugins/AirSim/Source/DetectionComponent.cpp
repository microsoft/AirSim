// Fill out your copyright notice in the Description page of Project Settings.


#include "DetectionComponent.h"

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <DrawDebugHelpers.h>
#include <Engine/Engine.h>
#include <Engine/StaticMesh.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Math/UnrealMathUtility.h>

UDetectionComponent::UDetectionComponent()
	: MaxDistanceToCamera(2000.f)
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}


void UDetectionComponent::BeginPlay()
{
	Super::BeginPlay();
	SceneCaptureComponent2D = CastChecked<USceneCaptureComponent2D> (GetAttachParent());
 	ObjectFilter = FObjectFilter();
}


void UDetectionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	CachedBoundingBoxes.Empty();

	for (TActorIterator<AActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{
		AActor* Actor = *ActorItr;
		if (ObjectFilter.MatchesActor(Actor))
		{
			if (FVector::/*DistSquared2D*/Distance(Actor->GetActorLocation(), GetComponentLocation()) <=
				/*MaxDistanceToCamera * */MaxDistanceToCamera)
			{
				FBox2D BoxOut;
				if (TextureTarget && CalcBoundingFromViewInfo(Actor, BoxOut))
				{
					CachedBoundingBoxes.Add(Actor,BoxOut);

					/*FVector Origin;
					FVector Extend;
					Actor->GetActorBounds(false, Origin, Extend);
					DrawDebugBox(GetWorld(), Origin, Extend, FColor::Red, false, 0.03, 0, 4.f);*/
				}
			}
		}		
	}
}

TMap<AActor*, FBox2D> UDetectionComponent::GetDetections() const
{
	return CachedBoundingBoxes;
}

bool UDetectionComponent::CalcBoundingFromViewInfo(AActor* Actor, FBox2D& BoxOut)
{
	FVector Origin;
	FVector Extend;
	Actor->GetActorBounds(false, Origin, Extend);
	
	TArray<FVector> Points;
	TArray<FVector2D> Points2D;
	bool IsInView = false;
	
	// get render target for texture size
	FRenderTarget* RenderTarget = TextureTarget->GameThread_GetRenderTargetResource();
	
	// initialize viewinfo for projection matrix
	FMinimalViewInfo Info; 
	Info.Location = SceneCaptureComponent2D->GetComponentTransform().GetLocation();
	Info.Rotation = SceneCaptureComponent2D->GetComponentTransform().GetRotation().Rotator();
	Info.FOV = SceneCaptureComponent2D->FOVAngle;
	Info.ProjectionMode = SceneCaptureComponent2D->ProjectionType;
	Info.AspectRatio = float(TextureTarget->SizeX) / float(TextureTarget->SizeY);
	Info.OrthoNearClipPlane = 1;
	Info.OrthoFarClipPlane = 0;//1000;
	Info.bConstrainAspectRatio = true;
	
	// calculate 3D corner Points of bounding box
	Points.Add(Origin + FVector(Extend.X, Extend.Y, Extend.Z));
	Points.Add(Origin + FVector(-Extend.X, Extend.Y, Extend.Z));
	Points.Add(Origin + FVector(Extend.X, -Extend.Y, Extend.Z));
	Points.Add(Origin + FVector(-Extend.X, -Extend.Y, Extend.Z));
	Points.Add(Origin + FVector(Extend.X, Extend.Y, -Extend.Z));
	Points.Add(Origin + FVector(-Extend.X, Extend.Y, -Extend.Z));
	Points.Add(Origin + FVector(Extend.X, -Extend.Y, -Extend.Z));
	Points.Add(Origin + FVector(-Extend.X, -Extend.Y, -Extend.Z));
	
	// initialize pixel values
	FVector2D MinPixel(TextureTarget->SizeX, TextureTarget->SizeY);
	FVector2D MaxPixel(0, 0);
	FIntRect ScreenRect(0, 0, TextureTarget->SizeX, TextureTarget->SizeY);
	
	// initialize projection data for sceneview
	FSceneViewProjectionData ProjectionData;
	ProjectionData.ViewOrigin = Info.Location;
	
	// do some voodoo rotation that is somehow mandatory and stolen from UGameplayStatics::ProjectWorldToScreen
	ProjectionData.ViewRotationMatrix = FInverseRotationMatrix(Info.Rotation) * FMatrix(
		FPlane(0, 0, 1, 0),
		FPlane(1, 0, 0, 0),
		FPlane(0, 1, 0, 0),
		FPlane(0, 0, 0, 1));

	if (SceneCaptureComponent2D->bUseCustomProjectionMatrix == true) 
	{
		ProjectionData.ProjectionMatrix = SceneCaptureComponent2D->CustomProjectionMatrix;
	}
	else 
	{
		ProjectionData.ProjectionMatrix = Info.CalculateProjectionMatrix();;
	}
	ProjectionData.SetConstrainedViewRectangle(ScreenRect);

	// Project Points to pixels and get the corner pixels
	for (FVector& Point : Points) 
	{
		FVector2D Pixel(0, 0);
		FSceneView::ProjectWorldToScreen((Point), ScreenRect, ProjectionData.ComputeViewProjectionMatrix(), Pixel);
		IsInView |= (Pixel != ScreenRect.Min) && (Pixel != ScreenRect.Max) && ScreenRect.Contains(FIntPoint(Pixel.X,Pixel.Y));
		Points2D.Add(Pixel);
		MaxPixel.X = FMath::Max(Pixel.X, MaxPixel.X);
		MaxPixel.Y = FMath::Max(Pixel.Y, MaxPixel.Y);
		MinPixel.X = FMath::Min(Pixel.X, MinPixel.X);
		MinPixel.Y = FMath::Min(Pixel.Y, MinPixel.Y);
	}

	FBox2D BoxOutTemp = FBox2D(MinPixel, MaxPixel);

	BoxOut.Min.X = FMath::Clamp<float>(BoxOutTemp.Min.X, 0, TextureTarget->SizeX);
	BoxOut.Min.Y = FMath::Clamp<float>(BoxOutTemp.Min.Y, 0, TextureTarget->SizeY);
	BoxOut.Max.X = FMath::Clamp<float>(BoxOutTemp.Max.X, 0, TextureTarget->SizeX);
	BoxOut.Max.Y = FMath::Clamp<float>(BoxOutTemp.Max.Y, 0, TextureTarget->SizeY);

	return IsInView;
}



