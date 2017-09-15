#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TaskGraphInterfaces.h"
#include "ImageUtils.h"

RenderRequest::RenderRequest(bool use_safe_method)
{
    data = std::make_shared<RenderRequestInfo>();
    data->use_safe_method = use_safe_method;
}
RenderRequest::~RenderRequest()
{
    data->render_target = nullptr;
    data = nullptr;
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(UTextureRenderTarget2D* renderTarget, TArray<uint8>& image_data_uint8, 
    TArray<float>& image_data_float, bool pixels_as_float, bool compress, int& width, int& height)
{
    data->render_target = renderTarget;
    if (!pixels_as_float)
        data->bmp.Reset();
    else
        data->bmp_float.Reset();
    data->pixels_as_float = pixels_as_float;
    data->compress = compress;

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (data->use_safe_method) {
        //TODO: below doesn't work right now because it must be running in game thread
        FIntPoint size;
        if (!data->pixels_as_float) {
            //below is documented method but more expensive because it forces flush
            FTextureRenderTargetResource* rt_resource = data->render_target->GameThread_GetRenderTargetResource();
            auto flags = setupRenderResource(rt_resource, data.get(), size);
            rt_resource->ReadPixels(data->bmp, flags);
        }
        else {
            FTextureRenderTargetResource* rt_resource = data->render_target->GetRenderTargetResource();
            setupRenderResource(rt_resource, data.get(), size);
            rt_resource->ReadFloat16Pixels(data->bmp_float);
        }
    }
    else {
        //wait for render thread to pick up our task

        // Queue up the task of rendering the scene in the render thread
        TGraphTask<RenderRequest>::CreateTask().ConstructAndDispatchWhenReady(*this);

        // wait for this task to complete
        if (!data->signal.waitFor(5)) {
            throw std::runtime_error("timeout waiting for screenshot");
        }
    }
    
    width = data->width;
    height = data->height;

    if (!pixels_as_float) {
        if (data->width != 0 && data->height != 0) {
            if (data->compress)
                FImageUtils::CompressImageArray(data->width, data->height, data->bmp, image_data_uint8);
            else {
                for (const auto& item : data->bmp) {
                    image_data_uint8.Add(item.R);
                    image_data_uint8.Add(item.G);
                    image_data_uint8.Add(item.B);
                    image_data_uint8.Add(item.A);
                }
            }
        }
    }
    else {
        for (const auto& item : data->bmp_float) {
            float fval = item.R.GetFloat();
            image_data_float.Add(fval);
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(FTextureRenderTargetResource* rt_resource, RenderRequestInfo* data, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    data->width = size.X;
    data->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (data != nullptr)
    {
        FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
        auto rt_resource = data->render_target->GetRenderTargetResource();
        if (rt_resource != nullptr) {
            const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
            FIntPoint size;
            auto flags = setupRenderResource(rt_resource, data.get(), size);

            //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
            //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
            if (!data->pixels_as_float) {
                //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                RHICmdList.ReadSurfaceData(
                    rhi_texture,
                    FIntRect(0, 0, size.X, size.Y),
                    data->bmp,
                    flags);
            }
            else {
                RHICmdList.ReadSurfaceFloatData(
                    rhi_texture,
                    FIntRect(0, 0, size.X, size.Y),
                    data->bmp_float,
                    CubeFace_PosX, 0, 0
                );
            }
        }

        data->signal.signal();
    }
}
