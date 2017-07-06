#pragma once
#include <AirSim.h>
#include <thread>
#include <mutex>
#include "ImageUtils.h"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "SimMode/SimModeWorldMultiRotor.h"
#include "CameraDirector.h"
#include "PIPCamera.h"
#include "MultiRotorConnector.h"
#include "SimMode/SimModeWorldBase.h"
#include "common/ClockFactory.hpp"

class FRecordingThread : public FRunnable
{
private:
    static FRecordingThread* Runnable;
    FRunnableThread* thread_;
    ASimModeBase* sim_mode_;
    const msr::airlib::PhysicsBody* fpv_physics_body_;

    FThreadSafeCounter stop_task_counter_;
    FRenderCommandFence read_pixel_fence_;
    FString image_path_;

public:
    FRecordingThread(FString imagePath, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body);
    virtual ~FRecordingThread();
    static FRecordingThread* ThreadInit(FString path, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body);
    static void Shutdown();

private:
    virtual bool Init();
    virtual uint32 Run();
    virtual void Stop();
    void SaveImage(TArray<uint8>& compressedPng);

    void EnsureCompletion();

    unsigned int images_saved_ = 0;

    msr::airlib::ClockBase* clock_ = msr::airlib::ClockFactory::get();
};