// Fill out your copyright notice in the Description page of Project Settings.


#include "DetectionComponent.h"

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/Engine.h>
#include <Engine/StaticMesh.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Kismet/KismetSystemLibrary.h>
#include <Runtime/Engine/Classes/Kismet/KismetRenderingLibrary.h>

UDetectionComponent::UDetectionComponent()
	: bOnlyTrackRecentlyRenderedActors(false)
	, bOnlyTrackOnScreenActors(false)
	, MinimalRequiredBoundingBoxSize(0, 0)
	, MaxDistanceToCamera(2000.f)
	, Header(TEXT(""))
	, FormatActorString(TEXT("{ActorName} {WorldLocation}"))
	, Separator(TEXT("\n"))
	, Footer(TEXT(""))
	, FormatVector2DString(TEXT("{X} {Y}"))
	, FormatVector3DString(TEXT("{X} {Y} {Z}"))
	, FormatRotatorString(TEXT("{Yaw} {Pitch} {Roll}"))
	, Format2DBoxString(TEXT("{Min} {Max} {Center} {Extent} {Width} {Height}"))
	, Format3DBoxString(TEXT("{Min} {Max} {Center} {Extent}"))
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;

	// ...
}


// Called when the game starts
void UDetectionComponent::BeginPlay()
{
	Super::BeginPlay();
	SceneCaptureComponent2D = CastChecked<USceneCaptureComponent2D> (GetAttachParent());
 	ObjectFilter = FObjectFilter();
// 	ObjectFilter.WildcardMeshNames.Add(FString(TEXT("Car_*")));
// 	ObjectFilter.WildcardMeshNames.Add(FString(TEXT("Stop_Sign_*")));
}


// Called every frame
void UDetectionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	bool y = this->IsActive();
	// GetTrackedaCtors
	TArray<AActor*> TrackedActors;

	for (TActorIterator<AActor> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{
		AActor* Actor = *ActorItr;
		//for (const FGTObjectFilter& ObjectFilter : TrackActorsThatMatchFilter)
		{
			if (ObjectFilter.MatchesActor(Actor))
			{
				if (FVector::DistSquared2D(Actor->GetActorLocation(), GetComponentLocation()) <=
					MaxDistanceToCamera * MaxDistanceToCamera)
				{
					float RecentlyRenderedThreshold = 10.f;
					// TODO link this to timed capture tirgger or just disable render time based
					// check
					if (!bOnlyTrackRecentlyRenderedActors /*||
						(bOnlyTrackRecentlyRenderedActors &&
							IsActorRenderedOnScreen(Actor, RecentlyRenderedThreshold))*/)
					{
						TrackedActors.Push(Actor);
					}
				}
			}
		}
	}

	// Generate result string
	FString Result = TEXT("");

	Result.Append(Header);

	for (AActor* TrackedActor : TrackedActors)
	{
		UStaticMeshComponent* MeshComp = Cast<UStaticMeshComponent>(
			TrackedActor->GetComponentByClass(UStaticMeshComponent::StaticClass()));

		FString MeshName(TEXT("NOMESHONTHISACTOR"));
		if (MeshComp)
		{
			MeshComp->GetStaticMesh()->GetName(MeshName);
		}

// 		UGTImageGeneratorBase* LinkedImageGeneratorComponent =
// 			Cast<UGTImageGeneratorBase>(LinkedImageGenerator.GetComponent(GetOwner()));

		// Screen Location
		FVector2D ScreenLocation(-1.f, -1.f);
		FVector2D ScreenLocationNormalized;
		FBox2D ScreenBoundingBox;
		FBox2D ScreenBoundingBoxNormalized;
		if (SceneCaptureComponent2D->TextureTarget)
		{
			CalcBoundingFromViewInfo(TrackedActor);

			ProjectToPixelLocation(
				TrackedActor->GetActorLocation(), ScreenLocation);

			ScreenLocationNormalized =
				NormalizePixelLocation(ScreenLocation);

			GetActorScreenBoundingBox(
				TrackedActor, ScreenBoundingBox);

			ScreenBoundingBoxNormalized.Min =
				NormalizePixelLocation(
					ScreenBoundingBox.Min);
			ScreenBoundingBoxNormalized.Max =
				NormalizePixelLocation(
					ScreenBoundingBox.Max);
		}

		TMap<FString, FStringFormatArg> GlobalProperties{
			{TEXT("WorldLocation"), Vector3DToFormattedString(TrackedActor->GetActorLocation())},
			{TEXT("WorldRotation"), RotatorToFormattedString(TrackedActor->GetActorRotation())},
			{TEXT("ScreenLocation"), Vector2DToFormattedString(ScreenLocation)},
			{TEXT("ScreenLocationNormalized"), Vector2DToFormattedString(ScreenLocationNormalized)},
			{TEXT("ScreenBoundingBox"), Box2DToFormattedString(ScreenBoundingBox)},
			{TEXT("ScreenBoundingBoxNormalized"),
			 Box2DToFormattedString(ScreenBoundingBoxNormalized)},
			{TEXT("ActorName"), TrackedActor->GetName()},
			{TEXT("MeshName"), MeshName} };

		FString ActorResult = FString::Format(*FormatActorString, GlobalProperties);

		for (const TPair<FString, FString>& ReplacePair : ReplaceStrings)
		{
			ActorResult.ReplaceInline(
				*ReplacePair.Key, *ReplacePair.Value, ESearchCase::CaseSensitive);
		}

		Result.Append(ActorResult);
		Result.Append(Separator);
	}

	Result.RemoveFromEnd(Separator);
	Result.Append(Footer);

	CurrentResult = Result.ReplaceEscapedCharWithChar();

/*
	const auto Data = FGTFileUtilities::StringToCharArray(CurrentResult);

	DataReadyDelegate.Broadcast(Data, TimeStamp);*/



	/*

	//return;
	if (SceneCaptureComponent2D->TextureTarget == nullptr)
	{
		return;
	}

	UObject* WorldContext = GetWorld();//SceneCaptureComponent2D;
	UCanvas* Canvas;
	FVector2D Size;
	FDrawToRenderTargetContext Context;


	UKismetRenderingLibrary::BeginDrawCanvasToRenderTarget(WorldContext, SceneCaptureComponent2D->TextureTarget, Canvas, Size, Context);
	if (Canvas) 
	{

		FCanvasBoxItem BoxItem(
			FVector2D(200, 200),
			FVector2D(80.0, 80.0));
		BoxItem.SetColor(FColor::Red);
		BoxItem.LineThickness = 5.f;
		Canvas->DrawItem(BoxItem);
		SceneCaptureComponent2D->TextureTarget->UpdateResource();
//		Canvas->K2_DrawBox(FVector2D(10,10), FVector2D(50,50), 5.0, FLinearColor::Red);

		/ *for (size_t i = 0; i < TrackedActors.Num(); i++)
		{
			FLinearColor color = (TargetColors.Num() > i ? TargetColors[i] : DefaultColor);
			FVector Origin, Extend;
			TArray<FVector> Points;
			TArray<FVector2D> Pixels;
			FBox2D Bounds;
			(TrackedActors[i])->GetActorBounds(false, Origin, Extend);
			if (UMyPixelUtility::calcBoundingFromViewInfo(this, Origin, Extend, Bounds, Points, Pixels)) {
				// not possible in UE 4.19:
				//FVector2D BoundExtend = (Bounds.Max - Bounds.Min) / 2.0;
				//Canvas->K2_DrawBox((Bounds.Min + BoundExtend), BoundExtend, LineThickness, color);
				FVector2D TopRight(Bounds.Max.X, Bounds.Min.Y);
				FVector2D BottomLeft(Bounds.Min.X, Bounds.Max.Y);
				Canvas->K2_DrawLine(Bounds.Min, TopRight, LineThickness, color);
				Canvas->K2_DrawLine(Bounds.Min, BottomLeft, LineThickness, color);
				Canvas->K2_DrawLine(Bounds.Max, TopRight, LineThickness, color);
				Canvas->K2_DrawLine(Bounds.Max, BottomLeft, LineThickness, color);
			}
		}* /
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("No Canvas could be created from TextureTarget"));
	}
	UKismetRenderingLibrary::EndDrawCanvasToRenderTarget(WorldContext, Context);*/
}

bool UDetectionComponent::GetActorScreenBoundingBox(
	AActor* InActor,
	FBox2D& OutBox)
{
	if (CachedBoundingBoxes.Contains(InActor))
	{
		OutBox = CachedBoundingBoxes[InActor];
		return true;
	}

	FVector ActorCenter;
	FVector ActorExtent;
	InActor->GetActorBounds(false, ActorCenter, ActorExtent);

	const FVector BoundsPointMapping[8] = { FVector(1, 1, 1),
										   FVector(1, 1, -1),
										   FVector(1, -1, 1),
										   FVector(1, -1, -1),
										   FVector(-1, 1, 1),
										   FVector(-1, 1, -1),
										   FVector(-1, -1, 1),
										   FVector(-1, -1, -1) };
	FBox2D ScreenBoundingBox(EForceInit::ForceInitToZero);

	for (uint8 BoundsPointItr = 0; BoundsPointItr < 8; BoundsPointItr++)
	{
		FVector2D ProjectedWorldLocation;

		bool bValidPixelLocation =
			ProjectToPixelLocation(
				ActorCenter + (BoundsPointMapping[BoundsPointItr] * ActorExtent),
				ProjectedWorldLocation);

		if (bValidPixelLocation)
		{
			ScreenBoundingBox += FVector2D(ProjectedWorldLocation.X, ProjectedWorldLocation.Y);
		}
	}

/*
	if (bAccurateBoundingBoxes && CachedSegmentation.IsValid())
	{
		FBox2D AccurateScreenBoundingBox(EForceInit::ForceInitToZero);

		TArray<FColor> SegmentColors =
			SegmentationSceneCapture->GetSegmentColorsUsedForActor(InActor);
		int TotalPixelOverlap = 0;
		for (const FColor& SegmentColor : SegmentColors)
		{
			for (int X = ScreenBoundingBox.Min.X; X <= ScreenBoundingBox.Max.X; X++)
			{
				for (int Y = ScreenBoundingBox.Min.Y; Y <= ScreenBoundingBox.Max.Y; Y++)
				{
					FColor ColorInSegmentationMap = CachedSegmentation.GetPixel(X, Y);
					if (SegmentColor == ColorInSegmentationMap)
					{
						AccurateScreenBoundingBox += FVector2D(X, Y);
						TotalPixelOverlap++;
					}
				}
			}
		}
		// Discard bbox if not enough pixels are detetected inside the original box
		// This will not work great if the detect object is a ring shaped or similiar
		// TODO make configurable
		if (TotalPixelOverlap <= ScreenBoundingBox.GetArea() / 8)
		{
			AccurateScreenBoundingBox = FBox2D(EForceInit::ForceInitToZero);
		}

		ScreenBoundingBox = AccurateScreenBoundingBox;
	}*/

	CachedBoundingBoxes.Add(InActor, ScreenBoundingBox);
	OutBox = ScreenBoundingBox;

	return true;
}

bool UDetectionComponent::ProjectToPixelLocation(
	const FVector& Location,
	FVector2D& OutPixelLocation) const
{
	FIntPoint CaptureSize;
	if (SceneCaptureComponent2D->TextureTarget)
	{
		CaptureSize =
			FIntPoint(SceneCaptureComponent2D->TextureTarget->GetSurfaceWidth(), SceneCaptureComponent2D->TextureTarget->GetSurfaceHeight());
	}
	else
	{
		CaptureSize = FIntPoint(512, 512);//Resolution;
	}

	const FTransform& Transform = GetComponentToWorld();
	FMatrix ViewMatrix = Transform.ToInverseMatrixWithScale();
	FVector ViewLocation = Transform.GetTranslation();

	// swap axis st. x=z,y=x,z=y (unreal coordinate space) so that z is up
	ViewMatrix =
		ViewMatrix *
		FMatrix(FPlane(0, 0, 1, 0), FPlane(1, 0, 0, 0), FPlane(0, 1, 0, 0), FPlane(0, 0, 0, 1));

	const float FOV = SceneCaptureComponent2D->FOVAngle * (float)PI / 360.0f;

	float XAxisMultiplier;
	float YAxisMultiplier;

	if (CaptureSize.X > CaptureSize.Y)
	{
		// if the viewport is wider than it is tall
		XAxisMultiplier = 1.0f;
		YAxisMultiplier = CaptureSize.X / (float)CaptureSize.Y;
	}
	else
	{
		// if the viewport is taller than it is wide
		XAxisMultiplier = CaptureSize.Y / (float)CaptureSize.X;
		YAxisMultiplier = 1.0f;
	}

	FMatrix ProjectionMatrix = FReversedZPerspectiveMatrix(
		FOV, FOV, XAxisMultiplier, YAxisMultiplier, GNearClippingPlane, GNearClippingPlane);

	FMatrix ViewProjectionMatrix = ViewMatrix * ProjectionMatrix;

	FVector4 ScreenPoint = ViewProjectionMatrix.TransformFVector4(FVector4(Location, 1));

	if (ScreenPoint.W > 0.0f)
	{
		float InvW = 1.0f / ScreenPoint.W;
		float Y = (GProjectionSignY > 0.0f) ? ScreenPoint.Y : 1.0f - ScreenPoint.Y;
		FIntRect ViewRect = FIntRect(0, 0, CaptureSize.X, CaptureSize.Y);
		OutPixelLocation = FVector2D(
			FMath::Clamp(
				ViewRect.Min.X + (0.5f + ScreenPoint.X * 0.5f * InvW) * ViewRect.Width(),
				0.f,
				(float)ViewRect.Width() - 1),
			FMath::Clamp(
				ViewRect.Min.Y + (0.5f - Y * 0.5f * InvW) * ViewRect.Height(),
				0.f,
				(float)ViewRect.Height() - 1));

		return true;
	}
	return false;
}

FVector2D UDetectionComponent::NormalizePixelLocation(const FVector2D& PixelLocation) const
{
	FIntPoint CaptureSize(SceneCaptureComponent2D->TextureTarget->GetSurfaceWidth(), SceneCaptureComponent2D->TextureTarget->GetSurfaceHeight());

	return FVector2D(
		FMath::Clamp(PixelLocation.X / CaptureSize.X, 0.f, 1.f),
		FMath::Clamp(PixelLocation.Y / CaptureSize.Y, 0.f, 1.f));
}


FString UDetectionComponent::Vector2DToFormattedString(const FVector2D& InVector)
{
	return FString::Format(
		*FormatVector2DString, { {TEXT("X"), InVector.X}, {TEXT("Y"), InVector.Y} });
}

FString UDetectionComponent::Vector3DToFormattedString(const FVector& InVector)
{
	return FString::Format(
		*FormatVector3DString,
		{ {TEXT("X"), InVector.X}, {TEXT("Y"), InVector.Y}, {TEXT("Z"), InVector.Z} });
}

FString UDetectionComponent::RotatorToFormattedString(const FRotator& InRotator)
{
	return FString::Format(
		*FormatRotatorString,
		{ {TEXT("Yaw"), InRotator.Yaw},
		 {TEXT("Pitch"), InRotator.Pitch},
		 {TEXT("Roll"), InRotator.Roll} });
}

FString UDetectionComponent::Box2DToFormattedString(const FBox2D& InBox)
{
	return FString::Format(
		*Format2DBoxString,
		{ {TEXT("Min"), Vector2DToFormattedString(InBox.Min)},
		 {TEXT("Max"), Vector2DToFormattedString(InBox.Max)},
		 {TEXT("Width"), InBox.GetSize().X},
		 {TEXT("Height"), InBox.GetSize().Y},
		 {TEXT("Center"), Vector2DToFormattedString(InBox.GetCenter())},
		 {TEXT("Extent"), Vector2DToFormattedString(InBox.GetExtent())} });
}

FString UDetectionComponent::Box3DToFormattedString(const FBox& InBox)
{
	return FString::Format(
		*Format3DBoxString,
		{ {TEXT("Min"), Vector3DToFormattedString(InBox.Min)},
		 {TEXT("Max"), Vector3DToFormattedString(InBox.Max)},
		 {TEXT("Center"), Vector3DToFormattedString(InBox.GetCenter())},
		 {TEXT("Extent"), Vector3DToFormattedString(InBox.GetExtent())} });
}

bool UDetectionComponent::CalcBoundingFromViewInfo(AActor* Actor)
{
	FVector Origin;
	FVector Extend;
	Actor->GetActorBounds(false, Origin, Extend);
	
	UKismetSystemLibrary::DrawDebugBox(GetWorld(), Origin, Extend, FLinearColor::Red, FRotator::ZeroRotator, 2.f, 4.f);

	FBox2D BoxOut;
	TArray<FVector> Points;
	TArray<FVector2D> Points2D;
	bool isCompletelyInView = true;
	// get render target for texture size
	USceneCaptureComponent2D* RenderComponent = SceneCaptureComponent2D;
	UTextureRenderTarget2D* RenderTexture = RenderComponent->TextureTarget;
	FRenderTarget* RenderTarget = RenderTexture->GameThread_GetRenderTargetResource();
	// initialise viewinfo for projection matrix
	FMinimalViewInfo Info; 
//	RenderComponent->GetCameraView(FApp::GetDeltaTime(), Info);

	Info.Location = RenderComponent->GetComponentTransform().GetLocation();
	Info.Rotation = RenderComponent->GetComponentTransform().GetRotation().Rotator();
	Info.FOV = RenderComponent->FOVAngle;
	Info.ProjectionMode = RenderComponent->ProjectionType;
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

	if (RenderComponent->bUseCustomProjectionMatrix == true) {
		ProjectionData.ProjectionMatrix = RenderComponent->CustomProjectionMatrix;
	}
	else {
		ProjectionData.ProjectionMatrix = Info.CalculateProjectionMatrix();;
	}
	ProjectionData.SetConstrainedViewRectangle(ScreenRect);
	// Project Points to pixels and get the corner pixels
	for (FVector& Point : Points) {
		FVector2D Pixel(0, 0);
		FSceneView::ProjectWorldToScreen((Point), ScreenRect, ProjectionData.ComputeViewProjectionMatrix(), Pixel);
		Points2D.Add(Pixel);
		MaxPixel.X = FMath::Max(Pixel.X, MaxPixel.X);
		MaxPixel.Y = FMath::Max(Pixel.Y, MaxPixel.Y);
		MinPixel.X = FMath::Min(Pixel.X, MinPixel.X);
		MinPixel.Y = FMath::Min(Pixel.Y, MinPixel.Y);
	}

	BoxOut = FBox2D(MinPixel, MaxPixel);
	// clamp min point
	if (BoxOut.Min.X < 0) {
		BoxOut.Min.X = 0;
		isCompletelyInView = false;
	}
	else if (BoxOut.Min.X > RenderTexture->SizeX) {
		BoxOut.Min.X = RenderTexture->SizeX;
		isCompletelyInView = false;
	}
	if (BoxOut.Min.Y < 0) {
		BoxOut.Min.Y = 0;
		isCompletelyInView = false;
	}
	else if (BoxOut.Min.Y > RenderTexture->SizeY) {
		BoxOut.Min.Y = RenderTexture->SizeY;
		isCompletelyInView = false;
	}
	// clamp max point
	if (BoxOut.Max.X > RenderTexture->SizeX) {
		BoxOut.Max.X = RenderTexture->SizeX;
		isCompletelyInView = false;
	}
	else if (BoxOut.Max.X < 0) {
		BoxOut.Max.X = 0;
		isCompletelyInView = false;
	}
	if (BoxOut.Max.Y > RenderTexture->SizeY) {
		BoxOut.Max.Y = RenderTexture->SizeY;
		isCompletelyInView = false;
	}
	else if (BoxOut.Max.Y < 0) {
		BoxOut.Max.Y = 0;
		isCompletelyInView = false;
	}
	return isCompletelyInView;
}