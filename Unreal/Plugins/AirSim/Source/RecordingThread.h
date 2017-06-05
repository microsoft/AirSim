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
    static FRecordingThread* Runnable;
    FRunnableThread* Thread;
    ASimModeWorldMultiRotor* GameThread;

    FThreadSafeCounter StopTaskCounter;
    FRenderCommandFence ReadPixelFence;
    FString imagePath;

public:
    FRecordingThread(FString imagePath, ASimModeWorldMultiRotor* GameThread);
    virtual ~FRecordingThread();
    static FRecordingThread* ThreadInit(FString path, ASimModeWorldMultiRotor* GameThread);
    static void Shutdown();

private:
    virtual bool Init();
    virtual uint32 Run();
    virtual void Stop();
    void SaveImage(TArray<uint8>& compressedPng);

    void EnsureCompletion();

    unsigned int imagesSaved = 0;

    msr::airlib::ClockBase* clock_ = msr::airlib::ClockFactory::get();
};