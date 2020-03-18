#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "UnrealString.h"
#include <thread>
#include <chrono>
#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(std::vector<uint8_t>& rgba_output) : rgba_output_(&rgba_output)
{}

RenderRequest::~RenderRequest()
{}

void RenderRequest::FastScreenshot()
{
	UAirBlueprintLib::RunCommandOnGameThread([this]() {
		fast_cap_done_ = false;
		fast_rt_resource_ = fast_param_.render_component->TextureTarget->GameThread_GetRenderTargetResource();
		fast_param_.render_component->CaptureScene();
		ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER(
			SceneDrawCompletion,
			RenderRequest *, This, this,
			{
				This->RenderThreadScreenshotTask();
			}
		);
	}, false);

	//Try to wait just long enough for the render thread to finish the capture.
	//Start with a long wait, then check for completion more frequently.
	for (int best_guess_cap_time_microseconds = 500; !fast_cap_done_; best_guess_cap_time_microseconds = 200)
		std::this_thread::sleep_for(std::chrono::microseconds(best_guess_cap_time_microseconds));
}

void RenderRequest::RenderThreadScreenshotTask()
{
	FRHITexture2D *fast_cap_texture = fast_rt_resource_->TextureRHI->GetTexture2D();
	EPixelFormat pixelFormat = fast_cap_texture->GetFormat();
	uint32 width = fast_cap_texture->GetSizeX();
	uint32 height = fast_cap_texture->GetSizeY();
	uint32 stride;
	auto *src = (const unsigned char*)RHILockTexture2D(fast_cap_texture, 0, RLM_ReadOnly, stride, false); // needs to be on render thread
	//fast_result_->image_data_uint8->Reset(fast_result_->image_data_uint8->Num());
	//fast_result_->image_data_uint8->Append(src, height * stride);
	//void *dest = static_cast<void*>(fast_result_->image_data_uint8->GetData());

	if (src)
		//rgba_output_->insert(rgba_output_->begin(), &src[0], &src[height * stride]);
		FMemory::BigBlockMemcpy(rgba_output_->data(), src, height * stride); //TODO MAYBE THIS IS THE SLOW PART
	RHIUnlockTexture2D(fast_cap_texture, 0, false);
	fast_cap_done_ = true;
}
