#pragma once

#include "AirBlueprintLib.h"
#include "VehicleCameraConnector.h"
#include "Recording/RecordingFile.h"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"

class FRecordingThread : public FRunnable
{
public:
    FRecordingThread();
    virtual ~FRecordingThread();
    static FRecordingThread* ThreadInit(msr::airlib::VehicleCameraBase* camera, RecordingFile* recording_file, const msr::airlib::PhysicsBody* fpv_physics_body, const RecordingSettings& settings);
    static void Shutdown();

private:
    virtual bool Init();
    virtual uint32 Run();
    virtual void Stop();

    void EnsureCompletion();

private:
    FString image_path_;
    msr::airlib::VehicleCameraBase* camera_;
    RecordingFile* recording_file_;
    const msr::airlib::PhysicsBody* fpv_physics_body_;

    static FRecordingThread* instance_;
    FRunnableThread* thread_;

    FThreadSafeCounter stop_task_counter_;
    FRenderCommandFence read_pixel_fence_;

    RecordingSettings settings_;

    msr::airlib::TTimePoint last_screenshot_on_;
    msr::airlib::Pose last_pose_;

    bool is_ready_;
};