#pragma once

#include "AirBlueprintLib.h"
#include "SimMode/SimModeBase.h"
#include "physics/PhysicsBody.hpp"


class FRecordingThread : public FRunnable
{
private:
    FString image_path_;
    ASimModeBase* sim_mode_;
    const msr::airlib::PhysicsBody* fpv_physics_body_;

    static FRecordingThread* Runnable;
    FRunnableThread* thread_;

    FThreadSafeCounter stop_task_counter_;
    FRenderCommandFence read_pixel_fence_;

public:
    FRecordingThread(FString imagePath, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body);
    virtual ~FRecordingThread();
    static FRecordingThread* ThreadInit(FString path, ASimModeBase* sim_mode, const msr::airlib::PhysicsBody* fpv_physics_body);
    static void Shutdown();

private:
    virtual bool Init();
    virtual uint32 Run();
    virtual void Stop();
    void SaveImage(TArray<uint8>& image_data);

    void EnsureCompletion();
};