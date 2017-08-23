#include "RecordingThread.h"
#include "HAL/RunnableThread.h"
#include <thread>
#include <mutex>
#include "TaskGraphInterfaces.h"
#include "RenderRequest.h"
#include "PIPCamera.h"


FRecordingThread* FRecordingThread::instance_ = NULL;

FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), camera_(nullptr), recording_file_(nullptr), fpv_physics_body_(nullptr), is_ready_(false)
{
    thread_ = FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal); // Windows default, possible to specify more priority
}


FRecordingThread* FRecordingThread::ThreadInit(msr::airlib::VehicleCameraBase* camera, RecordingFile* recording_file, const msr::airlib::PhysicsBody* fpv_physics_body, const RecordingSettings& settings)
{
    if (!instance_ && FPlatformProcess::SupportsMultithreading())
    {
        instance_ = new FRecordingThread();
        instance_->camera_ = camera;
        instance_->recording_file_ = recording_file;
        instance_->fpv_physics_body_ = fpv_physics_body;
        instance_->settings_ = settings;

        instance_->last_screenshot_on_ = 0;
        instance_->last_pose_ = msr::airlib::Pose();

        instance_->is_ready_ = true;
    }
    return instance_;
}

FRecordingThread::~FRecordingThread()
{
    delete thread_;
    thread_ = NULL;
}

bool FRecordingThread::Init()
{
    if (camera_ && recording_file_)
    {
        UAirBlueprintLib::LogMessage(TEXT("Initiated recording thread"), TEXT(""), LogDebugLevel::Success);
    }
    return true;
}

uint32 FRecordingThread::Run()
{
    while (stop_task_counter_.GetValue() == 0)
    {
        //make sire all vars are set up
        if (is_ready_) {
            bool interval_elapsed = msr::airlib::ClockFactory::get()->elapsedSince(last_screenshot_on_) > settings_.record_interval;
            bool is_pose_unequal = fpv_physics_body_ && last_pose_ != fpv_physics_body_->getKinematics().pose;
            if (interval_elapsed && (!settings_.record_on_move || is_pose_unequal))
            {
                last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();
                last_pose_ = fpv_physics_body_->getKinematics().pose;

                // todo: should we go as fast as possible, or should we limit this to a particular number of
                // frames per second?
                auto response = camera_->getImage(msr::airlib::VehicleCameraBase::ImageType::Scene, false, true);
                TArray<uint8_t> image_data;
                image_data.Append(response.image_data_uint8.data(), response.image_data_uint8.size());
                recording_file_->appendRecord(image_data, fpv_physics_body_);
            }
        }
    }
    return 0;
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
    if (instance_)
    {
        instance_->EnsureCompletion();
        delete instance_;
        instance_ = NULL;
    }
}