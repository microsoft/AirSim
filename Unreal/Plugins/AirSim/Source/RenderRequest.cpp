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

    uint32 stride; // The number of bytes per row of image data (may include padding)
    auto *src = (const uint8_t*)RHILockTexture2D(fast_cap_texture, 0, RLM_ReadOnly, stride, false); // needs to be on render thread

    if (src) {
        switch (pixelFormat) {
        case PF_B8G8R8A8:
            {
                result.pixels_as_float = false;

                /*
                * Calculate the exact image size from the width, height, and
                * number of components
                */
                result.pixels = buffer_pool_->GetBufferExactSize(
                    width * height * 3);
                // The stride should exactly fit the image.
                result.stride = result.width * 3;

                // Convert the data to BGR8
                uint8* dst = result.pixels->data();
                bgra8ToBgr8(dst, src, width, height, stride);

                break;
            }

        case PF_FloatRGBA:
            {
                result.pixels_as_float = true;

                /*
                * Calculate the exact image size from the width, height, number
                * of components, and size of each component
                */
                result.pixels_float = buffer_pool_float_->GetBufferExactSize(
                    width * height * sizeof(float));
                // The stride should exactly fit the image.
                result.stride = result.width * sizeof(float);

                /*
                * We're using a depth buffer, where the depth is stored in the R
                * component. FloatRGBA is misleading, as it should actually be
                * Float16RGBA, or HalfFloatRGBA, because each component is
                * represented by a 16 bit float and not a 32 bit float.
                */
                float* dst = result.pixels_float->data();
                rgba16fToDepthFloat(dst, src, width, height, stride);

                break;
            }

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

void RenderRequest::bgra8ToBgr8(
    uint8* dst,
    const uint8* src,
    uint32 width,
    uint32 height,
    uint32 stride)
{
    for (uint32 i = 0; i < width; i++)
    {
        for (uint32 j = 0; j < height; j++)
        {
            memcpy(
                &dst[(i + j * width) * 3],
                &src[i * 4 + j * stride],
                3 * sizeof(uint8));
        }
    }
}

void RenderRequest::rgba16fToDepthFloat(
    float* dst,
    const uint8* src,
    uint32 width,
    uint32 height,
    uint32 stride)
{
    for (uint32 i = 0; i < width; i++)
    {
        for (uint32 j = 0; j < height; j++)
        {
            // Decode the R component and store it in the array
            int index0 = i * 8 + j * stride;
            FFloat16 val;
            val.Encoded = src[index0 + 0] | (src[index0 + 1] << 8);
            dst[i + j * width] = val;
        }
    }
}
