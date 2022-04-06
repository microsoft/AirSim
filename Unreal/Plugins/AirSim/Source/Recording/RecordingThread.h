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
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

public:
    FRecordingThread();
    virtual ~FRecordingThread();

    void init();
    void startRecording(const RecordingSetting& settings,
                        const common_utils::UniqueValueMap<std::string, VehicleSimApiBase*>& vehicle_sim_apis);
    void stopRecording();
    void killRecording();
    bool isRecording();

protected:
    virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;
    virtual void Exit() override;

private:
    FThreadSafeCounter stop_task_counter_;

    std::unique_ptr<FRecordingThread> running_instance_;
    std::unique_ptr<FRecordingThread> finishing_instance_;
    msr::airlib::WorkerThreadSignal finishing_signal_;
    bool first_;

    static std::unique_ptr<FRecordingThread> instance_;

    std::unique_ptr<FRunnableThread> thread_;

    RecordingSetting settings_;
    std::unique_ptr<RecordingFile> recording_file_;
    common_utils::UniqueValueMap<std::string, VehicleSimApiBase*> vehicle_sim_apis_;
    std::unordered_map<std::string, const ImageCaptureBase*> image_captures_;
    std::unordered_map<std::string, msr::airlib::Pose> last_poses_;

    msr::airlib::TTimePoint last_screenshot_on_;

    bool is_ready_;
};