
#include <AirSim.h>
#include <thread>
#include <mutex>
#include "Recording/RecordingThread.h"
#include "TaskGraphInterfaces.h"
#include "RenderRequest.h"
#include "PIPCamera.h"


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
                    RenderRequest request(false);
                    int width, height;
                    request.getScreenshot(renderTarget, image_data, false, true, width, height);
                    SaveImage(image_data);
                }
            }
        }
    }
    return 0;
}

void FRecordingThread::SaveImage(TArray<uint8>& image_data)
{
    sim_mode_->getRecordingFile().appendRecord(image_path_, image_data, fpv_physics_body_);
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