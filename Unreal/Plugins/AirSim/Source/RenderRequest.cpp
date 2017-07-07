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
void RenderRequest::getScreenshot(UTextureRenderTarget2D* renderTarget, TArray<uint8>& image_data, 
    bool pixels_as_float, bool compress, int& width, int& height)
{
    data->render_target = renderTarget;
    data->bmp.Reset();
    data->bmp_float.Reset();
    data->pixels_as_float = pixels_as_float;
    data->compress = compress;

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    // Queue up the task of rendering the scene in the render thread
    TGraphTask<RenderRequest>::CreateTask().ConstructAndDispatchWhenReady(*this);

    // wait for this task to complete
    if (!data->signal.waitFor(5)) {
        throw std::runtime_error("timeout waiting for screenshot");
    }
    
    width = data->width;
    height = data->height;

    if (!pixels_as_float) {
        if (data->width != 0 && data->height != 0) {
            if (data->compress)
                FImageUtils::CompressImageArray(data->width, data->height, data->bmp, image_data);
            else {
                for (const auto& item : data->bmp) {
                    image_data.Add(item.R);
                    image_data.Add(item.G);
                    image_data.Add(item.B);
                    image_data.Add(item.A);
                }
            }
        }
    }
    else {
        for (const auto& item : data->bmp_float) {
            float fval = item.R.GetFloat();
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&fval);
            for (int i = 0; i < sizeof(float); ++i)
                image_data.Add(*(bytes + i));
        }
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
                FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
                flags.SetLinearToGamma(false);

                if (!data->pixels_as_float) {
                    RHICmdList.ReadSurfaceData(
                        textureRef,
                        FIntRect(0, 0, size.X, size.Y),
                        data->bmp,
                        flags
                    );
                }
                else {
                    RHICmdList.ReadSurfaceFloatData(
                        textureRef,
                        FIntRect(0, 0, size.X, size.Y),
                        data->bmp_float,
                        CubeFace_PosX, 0, 0
                    );
                }
            }
        }
        catch (std::exception& e) {
            unused(e);
        }
        data->signal.signal();
    }
}
