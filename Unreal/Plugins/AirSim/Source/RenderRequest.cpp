#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TaskGraphInterfaces.h"
#include "ImageUtils.h"

RenderRequest::RenderRequest(bool use_safe_method)
    : use_safe_method_(use_safe_method), params_(nullptr), results_(nullptr), req_size_(0), 
        wait_signal_(new msr::airlib::WorkerThreadSignal)
{
}

RenderRequest::~RenderRequest()
{

}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size)
{
    //TODO: is below really needed?
    for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (! params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->time_stamp = 0;
    }

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    if (use_safe_method_) {
        for (unsigned int i = 0; i < req_size; ++i) {
            //TODO: below doesn't work right now because it must be running in game thread
            FIntPoint img_size;
            if (! params[i]->pixels_as_float) {
                //below is documented method but more expensive because it forces flush
                FTextureRenderTargetResource* rt_resource = params[i]->render_target->GameThread_GetRenderTargetResource();
                auto flags = setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                rt_resource->ReadPixels(results[i]->bmp, flags);
            }
            else {
                FTextureRenderTargetResource* rt_resource = params[i]->render_target->GetRenderTargetResource();
                setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                rt_resource->ReadFloat16Pixels(results[i]->bmp_float);
            }
        }
    }
    else {
        //wait for render thread to pick up our task
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;


        // Queue up the task of rendering the scene in the render thread
        TGraphTask<RenderRequest>::CreateTask().ConstructAndDispatchWhenReady(*this);

        // wait for this task to complete
        if (! wait_signal_->waitFor(5)) {
            throw std::runtime_error("timeout waiting for screenshot");
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        if (! params[i]->pixels_as_float) {
            if (results[i]->width != 0 && results[i]->height != 0) {
                results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 4, false);
                if (params[i]->compress)
                    FImageUtils::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                else {
                    uint8* ptr = results[i]->image_data_uint8.GetData();
                    for (const auto& item : results[i]->bmp) {
                        *ptr++ = item.R;
                        *ptr++ = item.G;
                        *ptr++ = item.B;
                        *ptr++ = item.A;
                    }
                }
            }
        }
        else {
            results[i]->image_data_float.SetNumUninitialized(results[i]->width * results[i]->height);
            float* ptr = results[i]->image_data_float.GetData();
            for (const auto& item : results[i]->bmp_float) {
                *ptr++ = item.R.GetFloat();
            }
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr && req_size_ > 0)
    {
        for (unsigned int i = 0; i < req_size_; ++i) {
            FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
            auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
            if (rt_resource != nullptr) {
                const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                FIntPoint size;
                auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

                //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
                //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
                if (! params_[i]->pixels_as_float) {
                    //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                    RHICmdList.ReadSurfaceData(
                        rhi_texture,
                        FIntRect(0, 0, size.X, size.Y),
                        results_[i]->bmp,
                        flags);
                }
                else {
                    RHICmdList.ReadSurfaceFloatData(
                        rhi_texture,
                        FIntRect(0, 0, size.X, size.Y),
                        results_[i]->bmp_float,
                        CubeFace_PosX, 0, 0
                    );
                }
            }

            results_[i]->time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
        }

        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;

        wait_signal_->signal();
    }
}
