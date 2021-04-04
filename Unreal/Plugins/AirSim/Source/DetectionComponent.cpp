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
#include <Kismet/KismetSystemLibrary.h>
#include <Kismet/KismetMathLibrary.h>
#include <Engine/EngineTypes.h>

UDetectionComponent::UDetectionComponent()
    : MaxDistanceToCamera(20000.f)
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

    CachedDetections.Empty();

    for (TActorIterator<AActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
    {
        AActor* Actor = *ActorItr;
        if (ObjectFilter.MatchesActor(Actor))
        {
            if (FVector::Distance(Actor->GetActorLocation(), GetComponentLocation()) <= MaxDistanceToCamera)
            {
                FBox2D Box2DOut;
                if (TextureTarget && CalcBoundingFromViewInfo(Actor, Box2DOut))
                {
                    FDetectionInfo Detection;
                    Detection.Actor = Actor;
                    Detection.Box2D = Box2DOut;

                    FBox Box3D = Actor->GetComponentsBoundingBox(true);
                    Detection.Box3D = FBox(GetRelativeLocation(Box3D.Min), GetRelativeLocation(Box3D.Max));

                    Detection.RelativeTransform = FTransform(GetRelativeRotation(Actor->GetActorLocation(), Actor->GetActorRotation()),
                        GetRelativeLocation(Actor->GetActorLocation()));
                    CachedDetections.Add(Detection);
                }
            }
        }
    }
}

TArray<FDetectionInfo> UDetectionComponent::GetDetections() const
{
    return CachedDetections;
}

bool UDetectionComponent::CalcBoundingFromViewInfo(AActor* Actor, FBox2D& BoxOut)
{
    FVector Origin;
    FVector Extend;
    Actor->GetActorBounds(false, Origin, Extend);
    
    TArray<FVector> Points;
    TArray<FVector2D> Points2D;
    bool IsInCameraView = false;
    
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
    Info.OrthoFarClipPlane = 100000;
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

    if (SceneCaptureComponent2D->bUseCustomProjectionMatrix) 
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
        IsInCameraView |= (Pixel != ScreenRect.Min) && (Pixel != ScreenRect.Max) && ScreenRect.Contains(FIntPoint(Pixel.X,Pixel.Y));
        Points2D.Add(Pixel);
        MaxPixel.X = FMath::Max(Pixel.X, MaxPixel.X);
        MaxPixel.Y = FMath::Max(Pixel.Y, MaxPixel.Y);
        MinPixel.X = FMath::Min(Pixel.X, MinPixel.X);
        MinPixel.Y = FMath::Min(Pixel.Y, MinPixel.Y);
    }

    // If actor in camera view - check if it's actually visible or hidden
    // Check against 8 extend points
    bool IsVisible = false; 
    if (IsInCameraView)
    {
        FHitResult Result;
        bool bWorldHit;
        for (FVector& Point : Points)
        {
            bWorldHit = GetWorld()->LineTraceSingleByChannel(Result, GetComponentLocation(), Point, ECC_WorldStatic);
            if (bWorldHit)
            {
                if (Result.Actor == Actor)
                {
                    IsVisible = true;
                    break;
                }
            }
        }
        
        // If actor in camera view but didn't hit any point out of 8 extend points,
        // check against 10 random points
        if (!IsVisible)
        {
            for (int i = 0; i < 10; i++)
            {
                FVector Point = UKismetMathLibrary::RandomPointInBoundingBox(Origin, Extend);
                bWorldHit = GetWorld()->LineTraceSingleByChannel(Result, GetComponentLocation(), Point, ECC_WorldStatic);
                if (bWorldHit)
                {
                    if (Result.Actor == Actor)
                    {
                        IsVisible = true;
                        break;
                    }
                }
            }
        }
    }

    FBox2D BoxOutTemp = FBox2D(MinPixel, MaxPixel);

    BoxOut.Min.X = FMath::Clamp<float>(BoxOutTemp.Min.X, 0, TextureTarget->SizeX);
    BoxOut.Min.Y = FMath::Clamp<float>(BoxOutTemp.Min.Y, 0, TextureTarget->SizeY);
    BoxOut.Max.X = FMath::Clamp<float>(BoxOutTemp.Max.X, 0, TextureTarget->SizeX);
    BoxOut.Max.Y = FMath::Clamp<float>(BoxOutTemp.Max.Y, 0, TextureTarget->SizeY);

    return IsInCameraView && IsVisible;
}

FVector UDetectionComponent::GetRelativeLocation(FVector InLocation)
{
    return GetComponentTransform().InverseTransformPosition(InLocation);
}

FRotator UDetectionComponent::GetRelativeRotation(FVector InLocation, FRotator InRotation)
{
    FTransform CameraTransform(GetComponentRotation(), GetComponentLocation());
    FTransform RelativeObjectTransform = CameraTransform.GetRelativeTransform(FTransform(InRotation, InLocation));
    return RelativeObjectTransform.Rotator();
}

