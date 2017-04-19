
#include <AirSim.h>
#include "RecordingThread.h"

FRecordingThread* FRecordingThread::Runnable = NULL;

FRecordingThread::FRecordingThread(FString path, ASimModeWorldMultiRotor* AirSim)
	: GameThread(AirSim)
	, StopTaskCounter(0)
{
	imagePath = path;
	Thread = FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal); // Windows default, possible to specify more priority
}

FRecordingThread::~FRecordingThread()
{
	delete Thread;
	Thread = NULL;
}

bool FRecordingThread::Init()
{
	if (GameThread)
	{
		UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
	}
	return true;
}

void FRecordingThread::ReadPixelsNonBlocking(TArray<FColor>& bmp)
{
	// Obtain reference to game camera and its associated capture component
	
	APIPCamera* cam = GameThread->CameraDirector->getCamera(0);	
	USceneCaptureComponent2D* capture = cam->getCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SCENE, true);

	if (capture != nullptr) {
		if (capture->TextureTarget != nullptr) {
			FTextureRenderTargetResource* RenderResource = capture->TextureTarget->GetRenderTargetResource();
			if (RenderResource != nullptr) {
				width = capture->TextureTarget->GetSurfaceWidth();
				height = capture->TextureTarget->GetSurfaceHeight();

				// Read the render target surface data back.	
				struct FReadSurfaceContext
				{
					FRenderTarget* SrcRenderTarget;
					TArray<FColor>* OutData;
					FIntRect Rect;
					FReadSurfaceDataFlags Flags;
				};

				bmp.Reset();
				FReadSurfaceContext ReadSurfaceContext =
				{
					RenderResource,
					&bmp,
					FIntRect(0, 0, RenderResource->GetSizeXY().X, RenderResource->GetSizeXY().Y),
					FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
				};

				// Queue up the task of rendering the scene in the render thread

				ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER(
					ReadSurfaceCommand,
					FReadSurfaceContext, Context, ReadSurfaceContext,
					{
						RHICmdList.ReadSurfaceData(
							Context.SrcRenderTarget->GetRenderTargetTexture(),
							Context.Rect,
							*Context.OutData,
							Context.Flags
						);
					});
			}
		}
	}
}

uint32 FRecordingThread::Run()
{
	while (StopTaskCounter.GetValue() == 0)
	{
		if (!bReadPixelsStarted)
		{
			ReadPixelsNonBlocking(imageColor);
			bReadPixelsStarted = true;

			// Declare task graph 'RenderStatus' in order to check on the status of the queued up render command

			DECLARE_CYCLE_STAT(TEXT("FNullGraphTask.CheckRenderStatus"), STAT_FNullGraphTask_CheckRenderStatus, STATGROUP_TaskGraphTasks);
			RenderStatus = TGraphTask<FNullGraphTask>::CreateTask(NULL).ConstructAndDispatchWhenReady(GET_STATID(STAT_FNullGraphTask_CheckRenderStatus), ENamedThreads::RenderThread);
		}				

		if (bReadPixelsStarted && !RenderStatus.GetReference() || RenderStatus->IsComplete()) {
			bReadPixelsStarted = false;
			RenderStatus = NULL;

			TArray<uint8> compressedPng;
			FIntPoint dest(width, height);
			FImageUtils::CompressImageArray(dest.X, dest.Y, imageColor, compressedPng);
			FString filePath = imagePath + FString::FromInt(imagesSaved) + ".png";
			bool imageSavedOk = FFileHelper::SaveArrayToFile(compressedPng, *filePath);

			// If render command is complete, save image along with position and orientation

			if (!imageSavedOk)
				UAirBlueprintLib::LogMessage(TEXT("File save failed to:"), filePath, LogDebugLevel::Failure);
			else {
				auto physics_body = static_cast<msr::airlib::PhysicsBody*>(GameThread->fpv_vehicle_connector_->getPhysicsBody());
				auto kinematics = physics_body->getKinematics();

				GameThread->record_file << msr::airlib::Utils::getTimeSinceEpochMillis() << "\t";    
				GameThread->record_file << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
				GameThread->record_file << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
				GameThread->record_file << "\n";

				UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
				imagesSaved++;
			}
		}		
	}
	return 0;
}

void FRecordingThread::Stop()
{
	StopTaskCounter.Increment();
}

FRecordingThread* FRecordingThread::ThreadInit(FString path, ASimModeWorldMultiRotor* AirSim)
{
	if (!Runnable && FPlatformProcess::SupportsMultithreading())
	{
		Runnable = new FRecordingThread(path, AirSim);
	}
	return Runnable;
}

void FRecordingThread::EnsureCompletion()
{
	Stop();
	Thread->WaitForCompletion();
	UAirBlueprintLib::LogMessage(TEXT("Stopped recording thread"), TEXT(""), LogDebugLevel::Success);
}

void FRecordingThread::Shutdown()
{
	if (Runnable)
	{
		Runnable->EnsureCompletion();
		delete Runnable;
		Runnable = NULL;
	}
}