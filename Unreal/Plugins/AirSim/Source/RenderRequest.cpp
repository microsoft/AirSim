#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "Containers/UnrealString.h"
#include <thread>
#include <chrono>
#include "BufferPool.h"
#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(BufferPool<uint8_t> *buffer_pool, BufferPool<float> *buffer_pool_float)
    : buffer_pool_(buffer_pool), buffer_pool_float_(buffer_pool_float)
{}

RenderRequest::~RenderRequest()
{}

void RenderRequest::FastScreenshot()
{
    std::unique_lock<std::mutex> lck(mtx_);
    fast_cap_done_ = false;
    lck.unlock();

    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        fast_rt_resource_ = fast_param_.render_component->TextureTarget->GameThread_GetRenderTargetResource();
        fast_param_.render_component->CaptureScene();
        ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)([this](FRHICommandListImmediate& RHICmdList)
        {
            this->RenderThreadScreenshotTask(this->latest_result_);
        });
    }, false);

    // Wait just long enough for the render thread to finish the capture.
    lck.lock();
    while (!fast_cap_done_)
        cv_.wait(lck);
}

void RenderRequest::RenderThreadScreenshotTask(RenderRequest::RenderResult &result)
{
    FRHITexture2D *fast_cap_texture = fast_rt_resource_->TextureRHI->GetTexture2D();
    EPixelFormat pixelFormat = fast_cap_texture->GetFormat();
    uint32 width = fast_cap_texture->GetSizeX();
    uint32 height = fast_cap_texture->GetSizeY();

    result.width = width;
    result.height = height;
    result.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();

    uint32 stride;
    auto *src = (const uint8_t*)RHILockTexture2D(fast_cap_texture, 0, RLM_ReadOnly, stride, false); // needs to be on render thread
    
    result.stride = stride;
    size_t size = height * stride;

    UE_LOG(LogTemp, Warning, TEXT("stats: H: %d,  W: %d,  S: %d,  px_format: %d"),
                                    height, width, stride, pixelFormat);

    if (src) {
        switch (pixelFormat) {
        case PF_B8G8R8A8:
            result.pixels_as_float = false;
            result.pixels = buffer_pool_->GetBufferExactSize(size);
            FMemory::BigBlockMemcpy(result.pixels->data(), src, size);
            break;

        case PF_FloatRGBA:
            result.pixels_as_float = true;
            result.pixels_float = buffer_pool_float_->GetBufferExactSize(size);
            FMemory::BigBlockMemcpy(result.pixels_float->data(), src, size);
            break;

        default:
            UE_LOG(LogTemp, Warning, TEXT("Unexpected pixel format: %d"), pixelFormat);
            break;
        }
    }
    
    RHIUnlockTexture2D(fast_cap_texture, 0, false);

    std::unique_lock<std::mutex> lck(mtx_);
    fast_cap_done_ = true;
    cv_.notify_all();
}
