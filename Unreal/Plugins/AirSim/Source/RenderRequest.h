#pragma once
#include <AirSim.h>
#include "common/common_utils/WorkerThread.hpp"

class RenderRequest : public FRenderCommand
{
    struct RenderRequestInfo {
        TArray<FColor> bmp;
        int width;
        int height;
        UTextureRenderTarget2D* render_target;
        msr::airlib::WorkerThreadSignal signal;
    };
    std::shared_ptr<RenderRequestInfo> data;
public:
    RenderRequest();
    RenderRequest(RenderRequest& other);
    ~RenderRequest();

    void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
    {
        ExecuteTask();
	} 

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
	}

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(UTextureRenderTarget2D* renderTarget, TArray<uint8>& compressedPng);
    void ExecuteTask();
};