#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "Containers/UnrealString.h"
#include <thread>
#include <chrono>
#include "common/common_utils/BufferPool.h"
#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(BufferPool *buffer_pool) : buffer_pool_(buffer_pool)
{}

RenderRequest::~RenderRequest()
{}

void RenderRequest::FastScreenshot()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        fast_cap_done_ = false;
        fast_rt_resource_ = fast_param_.render_component->TextureTarget->GameThread_GetRenderTargetResource();
        fast_param_.render_component->CaptureScene();
        ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)([this](FRHICommandListImmediate& RHICmdList)
        {
            this->RenderThreadScreenshotTask(this->latest_result_);
        });
    }, false);

    //Try to wait just long enough for the render thread to finish the capture.
    //Start with a long wait, then check for completion more frequently.
    //TODO: Optimize these numbers.
    for (int best_guess_cap_time_microseconds = 500; !fast_cap_done_; best_guess_cap_time_microseconds = 200)
        std::this_thread::sleep_for(std::chrono::microseconds(best_guess_cap_time_microseconds));
}

void RenderRequest::RenderThreadScreenshotTask(RenderRequest::RenderResult &result)
{
    FRHITexture2D *fast_cap_texture = fast_rt_resource_->TextureRHI->GetTexture2D();
    EPixelFormat pixelFormat = fast_cap_texture->GetFormat();
    uint32 width = fast_cap_texture->GetSizeX();
    uint32 height = fast_cap_texture->GetSizeY();
    uint32 stride;
    auto *src = (const unsigned char*)RHILockTexture2D(fast_cap_texture, 0, RLM_ReadOnly, stride, false); // needs to be on render thread
    
    result.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
    result.pixels = buffer_pool_->GetBufferExactSize(height*stride);
    result.stride = stride;
    result.width = width;
    result.height = height;

    if (src) {
        switch (pixelFormat) {
        case PF_B8G8R8A8:
            result.pixels_as_float = false;
            //response.image_data_uint8 = std::move(render_request.latest_result_.pixels);
            FMemory::BigBlockMemcpy(latest_result_.pixels->data(), src, height * stride);
            break;

        case PF_FloatRGBA:
            result.pixels_as_float = true;
            //response.image_data_float = std::move(render_request.latest_result_.pixels);
            FMemory::BigBlockMemcpy(latest_result_.pixels_float->data(), src, height * stride);
            break;

        default:
            UE_LOG(LogTemp, Warning, TEXT("Unexpected pixel format: %d"), pixelFormat);
            break;
        }
    }
		//FMemory::BigBlockMemcpy(latest_result_.pixels->data(), src, height * stride);
    
    RHIUnlockTexture2D(fast_cap_texture, 0, false);

    fast_cap_done_ = true;
}
