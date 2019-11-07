#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"

#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "Recording/RecordingFile.h"
#include "physics/Kinematics.hpp"
#include <memory>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"
#include "common/WorkerThread.hpp"

class FRecordingThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::RecordingSetting RecordingSetting;

public:
    FRecordingThread();
    virtual ~FRecordingThread();

    static void init();
    static void startRecording(const msr::airlib::ImageCaptureBase* camera, const msr::airlib::Kinematics::State* kinematics, 
        const RecordingSetting& settings, msr::airlib::VehicleSimApiBase* vehicle_sim_api);
    static void stopRecording();
    static void killRecording();
    static bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    FThreadSafeCounter stop_task_counter_;
    FRenderCommandFence read_pixel_fence_;

    static std::unique_ptr<FRecordingThread> running_instance_;
    static std::unique_ptr<FRecordingThread> finishing_instance_;
    static msr::airlib::WorkerThreadSignal finishing_signal_;
    static bool first_;
    
    static std::unique_ptr<FRecordingThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSetting settings_;
    const msr::airlib::ImageCaptureBase* image_capture_;
    std::unique_ptr<RecordingFile> recording_file_;
    const msr::airlib::Kinematics::State* kinematics_;
    msr::airlib::VehicleSimApiBase* vehicle_sim_api_;

    msr::airlib::TTimePoint last_screenshot_on_;
    msr::airlib::Pose last_pose_;

    bool is_ready_;
};