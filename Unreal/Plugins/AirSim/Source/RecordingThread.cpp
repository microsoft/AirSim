
#include <AirSim.h>
#include "RecordingThread.h"
#include "TaskGraphInterfaces.h"
#include "RenderRequest.h"

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

uint32 FRecordingThread::Run()
{
    while (StopTaskCounter.GetValue() == 0)
    {
        APIPCamera* cam = GameThread->CameraDirector->getCamera(0);
        if (cam != nullptr)
        {
            // todo: should we go as fast as possible, or should we limit this to a particular number of
            // frames per second?
            USceneCaptureComponent2D* capture = cam->getCaptureComponent(EPIPCameraType::PIP_CAMERA_TYPE_SCENE, true);
            if (capture != nullptr) {
                UTextureRenderTarget2D* renderTarget = capture->TextureTarget;
                if (renderTarget != nullptr) {
                    TArray<uint8> png;
                    RenderRequest request;
                    request.getScreenshot(renderTarget, png);
                    SaveImage(png);
                }
            }
        }
    }
    return 0;
}

void FRecordingThread::SaveImage(TArray<uint8>& compressedPng)
{
    if (compressedPng.Num() > 0) {
        FString filePath = imagePath + FString::FromInt(imagesSaved) + ".png";
        bool imageSavedOk = FFileHelper::SaveArrayToFile(compressedPng, *filePath);

        // If render command is complete, save image along with position and orientation

        if (!imageSavedOk)
            UAirBlueprintLib::LogMessage(TEXT("FAILED to save screenshot to:"), filePath, LogDebugLevel::Failure);
        else {
            auto physics_body = static_cast<msr::airlib::PhysicsBody*>(GameThread->fpv_vehicle_connector_->getPhysicsBody());
            auto kinematics = physics_body->getKinematics();

            uint64_t timestamp_millis = static_cast<uint64_t>(clock_->nowNanos() / 1.0E6);

            GameThread->record_file << timestamp_millis << "\t";
            GameThread->record_file << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
            GameThread->record_file << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
            GameThread->record_file << "\n";

            UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
            imagesSaved++;
        }
    }
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