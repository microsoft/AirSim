// Fill out your copyright notice in the Description page of Project Settings.


#include "DetectionComponent.h"

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/Engine.h>
#include <Engine/StaticMesh.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Kismet/KismetSystemLibrary.h>
//#include <Runtime/Engine/Classes/Kismet/KismetRenderingLibrary.h>
#include <Logging/LogMacros.h>
#include <Kismet/GameplayStatics.h>
#include <GameFramework/HUD.h>

UDetectionComponent::UDetectionComponent()
	: bOnlyTrackRecentlyRenderedActors(false)
	, bOnlyTrackOnScreenActors(false)
	, MaxDistanceToCamera(2000.f)
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
			if (FVector::DistSquared2D(Actor->GetActorLocation(), GetComponentLocation()) <=
				MaxDistanceToCamera * MaxDistanceToCamera)
			{
				FBox2D BoxOut;
				if (SceneCaptureComponent2D->TextureTarget && CalcBoundingFromViewInfo(Actor, BoxOut))
				{
					CachedBoundingBoxes.Add(Actor,BoxOut);

					FVector Origin;
					FVector Extend;
					Actor->GetActorBounds(false, Origin, Extend);
					UKismetSystemLibrary::DrawDebugBox(GetWorld(), Origin, Extend, FLinearColor::Red, FRotator::ZeroRotator, 0.1f, 4.f);
// 					int32 X;
// 					int32 Y;
// 					UGameplayStatics::GetPlayerControllerFromID(GetWorld(), 0)->GetViewportSize(X, Y);
// 					float XAxisMultiplier = X / SceneCaptureComponent2D->TextureTarget->GetSurfaceWidth();
// 					float YAxisMultiplier = Y / SceneCaptureComponent2D->TextureTarget->GetSurfaceHeight();
					//UGameplayStatics::GetPlayerControllerFromID(GetWorld(), 0)->GetHUD()->DrawRect(FLinearColor::Blue, BoxOut.Min.X * XAxisMultiplier, BoxOut.Min.Y * YAxisMultiplier, BoxOut.Max.X * XAxisMultiplier, BoxOut.Max.Y * YAxisMultiplier);					
					//UE_LOG(LogTemp, Log, TEXT("Object detected: %s"), *(Actor->GetFName().ToString()));
				}
			}
		}		
	}
}

bool UDetectionComponent::CalcBoundingFromViewInfo(AActor* Actor, FBox2D& BoxOut)
{
	FVector Origin;
	FVector Extend;
	Actor->GetActorBounds(false, Origin, Extend);
	
	/*FBox2D BoxOut;*/
	TArray<FVector> Points;
	TArray<FVector2D> Points2D;
	bool IsInView = false;
	
	// get render target for texture size
	UTextureRenderTarget2D* RenderTexture = SceneCaptureComponent2D->TextureTarget;
	FRenderTarget* RenderTarget = SceneCaptureComponent2D->TextureTarget->GameThread_GetRenderTargetResource();
	
	// initialize viewinfo for projection matrix
	FMinimalViewInfo Info; 
	Info.Location = SceneCaptureComponent2D->GetComponentTransform().GetLocation();
	Info.Rotation = SceneCaptureComponent2D->GetComponentTransform().GetRotation().Rotator();
	Info.FOV = SceneCaptureComponent2D->FOVAngle;
	Info.ProjectionMode = SceneCaptureComponent2D->ProjectionType;
	Info.AspectRatio = float(RenderTexture->SizeX) / float(RenderTexture->SizeY);
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
	FVector2D MinPixel(RenderTexture->SizeX, RenderTexture->SizeY);
	FVector2D MaxPixel(0, 0);
	FIntRect ScreenRect(0, 0, RenderTexture->SizeX, RenderTexture->SizeY);
	
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

	BoxOut = FBox2D(MinPixel, MaxPixel);
	// clamp min point
	if (BoxOut.Min.X < 0) 
	{
		BoxOut.Min.X = 0;
	//	IsInView = false;
	}
	else if (BoxOut.Min.X > RenderTexture->SizeX) 
	{
		BoxOut.Min.X = RenderTexture->SizeX;
	//	IsInView = false;
	}
	if (BoxOut.Min.Y < 0) 
	{
		BoxOut.Min.Y = 0;
	//	IsInView = false;
	}
	else if (BoxOut.Min.Y > RenderTexture->SizeY) 
	{
		BoxOut.Min.Y = RenderTexture->SizeY;
//		IsInView = false;
	}
	// clamp max point
	if (BoxOut.Max.X > RenderTexture->SizeX) 
	{
		BoxOut.Max.X = RenderTexture->SizeX;
	//	IsInView = false;
	}
	else if (BoxOut.Max.X < 0) 
	{
		BoxOut.Max.X = 0;
//		IsInView = false;
	}
	if (BoxOut.Max.Y > RenderTexture->SizeY) 
	{
		BoxOut.Max.Y = RenderTexture->SizeY;
//		IsInView = false;
	}
	else if (BoxOut.Max.Y < 0) 
	{
		BoxOut.Max.Y = 0;
	//	IsInView = false;
	}
	return IsInView;
}



