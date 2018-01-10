#pragma once

#include "CoreMinimal.h"
#include "AirBlueprintLib.h"
#include "HAL/Runnable.h"
#include "UnrealImageCapture.h"
#include "VehiclePawnWrapper.h"
#include "Recording/RecordingFile.h"
#include "physics/Kinematics.hpp"
#include <memory>
#include "common/ClockFactory.hpp"
#include "common/AirSimSettings.hpp"

class FRecordingThread : public FRunnable
{
public:
    typedef msr::airlib::AirSimSettings::RecordingSettings RecordingSettings;

public:
    FRecordingThread();
    virtual ~FRecordingThread();
    static void startRecording(msr::airlib::ImageCaptureBase* camera, const msr::airlib::Kinematics::State* kinematics, 
        const RecordingSettings& settings, VehiclePawnWrapper* wrapper);
    static void stopRecording(); 
    static bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    void EnsureCompletion();

private:
    FThreadSafeCounter stop_task_counter_;
    FRenderCommandFence read_pixel_fence_;
    
    static std::unique_ptr<FRecordingThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSettings settings_;
    msr::airlib::ImageCaptureBase* image_capture_;
    std::unique_ptr<RecordingFile> recording_file_;
    const msr::airlib::Kinematics::State* kinematics_;
    VehiclePawnWrapper* wrapper_;

    msr::airlib::TTimePoint last_screenshot_on_;
    msr::airlib::Pose last_pose_;

    bool is_ready_;
};