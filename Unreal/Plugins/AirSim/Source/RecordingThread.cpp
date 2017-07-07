
#include <AirSim.h>
#include "RecordingThread.h"
#include "TaskGraphInterfaces.h"
#include "RenderRequest.h"

FRecordingThread* FRecordingThread::Runnable = NULL;

FRecordingThread::FRecordingThread(FString path, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body)
    : image_path_(path), sim_mode_(sim_mode), fpv_physics_body_(fpv_physics_body), stop_task_counter_(0)
{
    thread_ = FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal); // Windows default, possible to specify more priority
}


FRecordingThread* FRecordingThread::ThreadInit(FString path, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body)
{
    if (!Runnable && FPlatformProcess::SupportsMultithreading())
    {
        Runnable = new FRecordingThread(path, sim_mode, fpv_physics_body);
    }
    return Runnable;
}

FRecordingThread::~FRecordingThread()
{
    delete thread_;
    thread_ = NULL;
}

bool FRecordingThread::Init()
{
    if (sim_mode_)
    {
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }
    return true;
}

uint32 FRecordingThread::Run()
{
    while (stop_task_counter_.GetValue() == 0)
    {
        APIPCamera* cam = sim_mode_->getFpvVehiclePawn()->getCamera();
        if (cam != nullptr)
        {
            // todo: should we go as fast as possible, or should we limit this to a particular number of
            // frames per second?
            USceneCaptureComponent2D* capture = cam->getCaptureComponent(msr::airlib::VehicleCameraBase::ImageType_::Scene, false);
            if (capture != nullptr) {
                UTextureRenderTarget2D* renderTarget = capture->TextureTarget;
                if (renderTarget != nullptr) {
                    TArray<uint8> image_data;
                    RenderRequest request;
                    request.getScreenshot(renderTarget, image_data);
                    SaveImage(image_data);
                }
            }
        }
    }
    return 0;
}

void FRecordingThread::SaveImage(TArray<uint8>& compressedPng)
{
    if (compressedPng.Num() > 0) {
        FString filePath = image_path_ + FString::FromInt(images_saved_) + ".png";
        bool imageSavedOk = FFileHelper::SaveArrayToFile(compressedPng, *filePath);

        // If render command is complete, save image along with position and orientation

        if (!imageSavedOk)
            UAirBlueprintLib::LogMessage(TEXT("FAILED to save screenshot to:"), filePath, LogDebugLevel::Failure);
        else {
            auto kinematics = fpv_physics_body_->getKinematics();

            uint64_t timestamp_millis = static_cast<uint64_t>(clock_->nowNanos() / 1.0E6);

            sim_mode_->record_file << timestamp_millis << "\t";
            sim_mode_->record_file << kinematics.pose.position.x() << "\t" << kinematics.pose.position.y() << "\t" << kinematics.pose.position.z() << "\t";
            sim_mode_->record_file << kinematics.pose.orientation.w() << "\t" << kinematics.pose.orientation.x() << "\t" << kinematics.pose.orientation.y() << "\t" << kinematics.pose.orientation.z() << "\t";
            sim_mode_->record_file << "\n";

            UAirBlueprintLib::LogMessage(TEXT("Screenshot saved to:"), filePath, LogDebugLevel::Success);
            images_saved_++;
        }
    }
}

void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
}


void FRecordingThread::EnsureCompletion()
{
    Stop();
    thread_->WaitForCompletion();
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