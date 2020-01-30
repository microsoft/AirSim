#include "RecordingThread.h"
#include "Async/TaskGraphInterfaces.h"
#include "HAL/RunnableThread.h"

#include <thread>
#include <mutex>
#include "RenderRequest.h"
#include "PIPCamera.h"


std::unique_ptr<FRecordingThread> FRecordingThread::running_instance_;
std::unique_ptr<FRecordingThread> FRecordingThread::finishing_instance_;
msr::airlib::WorkerThreadSignal FRecordingThread::finishing_signal_;
bool FRecordingThread::first_ = true;


FRecordingThread::FRecordingThread()
    : stop_task_counter_(0), recording_file_(nullptr), kinematics_(nullptr), is_ready_(false)
{
    thread_.reset(FRunnableThread::Create(this, TEXT("FRecordingThread"), 0, TPri_BelowNormal)); // Windows default, possible to specify more priority
}


void FRecordingThread::startRecording(const msr::airlib::ImageCaptureBase* image_capture, const msr::airlib::Kinematics::State* kinematics,
    const RecordingSetting& settings, msr::airlib::VehicleSimApiBase* vehicle_sim_api)
{
    stopRecording();

    //TODO: check FPlatformProcess::SupportsMultithreading()?
    assert(!isRecording());

    running_instance_.reset(new FRecordingThread());
    running_instance_->image_capture_ = image_capture;
    running_instance_->kinematics_ = kinematics;
    running_instance_->settings_ = settings;
    running_instance_->vehicle_sim_api_ = vehicle_sim_api;

    running_instance_->last_screenshot_on_ = 0;
    running_instance_->last_pose_ = msr::airlib::Pose();

    running_instance_->is_ready_ = true;

    running_instance_->recording_file_.reset(new RecordingFile());
    running_instance_->recording_file_->startRecording(vehicle_sim_api);
}

FRecordingThread::~FRecordingThread()
{
    if (this == running_instance_.get()) stopRecording();
}

void FRecordingThread::init()
{
    first_ = true;
}

bool FRecordingThread::isRecording()
{
    return running_instance_ != nullptr;
}

void FRecordingThread::stopRecording()
{
    if (running_instance_)
    {
        assert(finishing_instance_ == nullptr);
        finishing_instance_ = std::move(running_instance_);
        assert(!isRecording());
        finishing_instance_->Stop();
    }
}

void FRecordingThread::killRecording()
{
    if (first_) return;

    stopRecording();
    bool finished = finishing_signal_.waitForRetry(1, 5);
    if (!finished) {
        UE_LOG(LogTemp, Log, TEXT("killing thread"));
        finishing_instance_->thread_->Kill(false);
    }
}

/*********************** methods for instance **************************************/

bool FRecordingThread::Init()
{
    if (first_) {
        first_ = false;
    }
    else {
        finishing_signal_.wait();
    }
    if (image_capture_ && recording_file_)
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
            bool is_pose_unequal = kinematics_ && last_pose_ != kinematics_->pose;
            if (interval_elapsed && (!settings_.record_on_move || is_pose_unequal))
            {
                last_screenshot_on_ = msr::airlib::ClockFactory::get()->nowNanos();
                last_pose_ = kinematics_->pose;

                //TODO: should we go as fast as possible, or should we limit this to a particular number of
                // frames per second?

                //BG: Workaround to get sync ground truth. See https://github.com/Microsoft/AirSim/issues/1494 for details
                uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);
                std::string gt = vehicle_sim_api_->getRecordFileLine(false);
                std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses;

                image_capture_->getImages(settings_.requests, responses);
                recording_file_->appendRecord(responses, vehicle_sim_api_);
            }
        }
    }

    recording_file_.reset();

    return 0;
}


void FRecordingThread::Stop()
{
    stop_task_counter_.Increment();
}


void FRecordingThread::Exit()
{
    assert(this == finishing_instance_.get());
    if (recording_file_)
        recording_file_.reset();
    finishing_signal_.signal();
}
