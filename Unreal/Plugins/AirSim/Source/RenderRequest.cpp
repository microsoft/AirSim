#include <AirSim.h>
#include "TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "RenderRequest.h"

RenderRequest::RenderRequest()
{
    data = std::make_shared<RenderRequestInfo>();
}
RenderRequest::RenderRequest(RenderRequest& other) {
    data = other.data;
}
RenderRequest::~RenderRequest()
{
    data->render_target = nullptr;
    data = nullptr;
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(UTextureRenderTarget2D* renderTarget, TArray<uint8>& compressedPng)
{
    data->render_target = renderTarget;
    data->bmp.Reset();

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();
    // Queue up the task of rendering the scene in the render thread

    TGraphTask<RenderRequest>::CreateTask().ConstructAndDispatchWhenReady(*this);

    // wait for this task to complete
    if (!data->signal.waitFor(5)) {
        throw std::runtime_error("timeout waiting for screenshot");
    }

    if (data->width != 0 && data->height != 0) {
        FImageUtils::CompressImageArray(data->width, data->height, data->bmp, compressedPng);
    }
}

void RenderRequest::ExecuteTask()
{
    if (data != nullptr)
    {
        try {
            FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
            auto resource = data->render_target->GetRenderTargetResource();
            if (resource != nullptr) {
                const FTexture2DRHIRef& textureRef = resource->GetRenderTargetTexture();
                auto size = resource->GetSizeXY();
                data->width = size.X;
                data->height = size.Y;
                RHICmdList.ReadSurfaceData(
                    textureRef,
                    FIntRect(0, 0, size.X, size.Y),
                    data->bmp,
                    FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
                );
            }
        }
        catch (std::exception& e) {
            unused(e);
        }
        data->signal.signal();
    }
}
