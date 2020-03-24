#include "UnrealImageCapture.h"
#include "Engine/World.h"
#include "ImageUtils.h"
#include "RenderRequest.h"
#include "common/ClockFactory.hpp"

UnrealImageCapture::UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
	: cameras_(cameras)
{
	//TODO: explore screenshot option
	//addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
	for (int i = 0; i < requests.size() && i < responses.size(); ++i)
		getImage(requests[i], responses[i]);
}

void UnrealImageCapture::getImage(const msr::airlib::ImageCaptureBase::ImageRequest& request, msr::airlib::ImageCaptureBase::ImageResponse& response) const
{
	getSceneCaptureImage(request.camera_name, request.image_type, response);
}

void UnrealImageCapture::getSceneCaptureImage(const std::string& camera_name, msr::airlib::ImageCaptureBase::ImageType image_type, msr::airlib::ImageCaptureBase::ImageResponse& response) const
{
	APIPCamera* camera = cameras_->at(camera_name);
	camera->setCameraTypeEnabled(image_type, true);

	USceneCaptureComponent2D* capture = camera->getCaptureComponent(image_type, false);
	UTextureRenderTarget2D* textureTarget = capture->TextureTarget;

	int height = capture->TextureTarget->SizeY;
	int width = capture->TextureTarget->SizeX;
	unsigned long log2;
	_BitScanReverse(&log2, width - 1);
	unsigned long long stride = 1ULL << (log2 + 1); //Round up to nearest power of 2.
	response.image_data_uint8 = std::move(BufferPool_->GetBufferExactSize(height * stride * 4));
	//TODO check not nullptr
	RenderRequest render_request(*response.image_data_uint8);
	render_request.fast_param_ = RenderRequest::RenderParams{ capture, textureTarget, false, false }; //render_params.at(0).get();
	render_request.FastScreenshot();

	response.time_stamp = render_request.latest_result_.time_stamp;
	//response.image_data_uint8 = std::vector<uint8_t>(result->image_data_uint8->GetData(), result->image_data_uint8->GetData() + result->image_data_uint8->Num());
	response.width = width;
	response.height = height;
	response.image_type = image_type;
}

bool UnrealImageCapture::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
	FScreenshotRequest::RequestScreenshot(false); // This is an async operation
	return true;
}

void UnrealImageCapture::addScreenCaptureHandler(UWorld *world)
{
	static bool is_installed = false;

	if (!is_installed) {
		UGameViewportClient* ViewportClient = world->GetGameViewport();
		ViewportClient->OnScreenshotCaptured().Clear();
		ViewportClient->OnScreenshotCaptured().AddLambda(
			[this](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap)
		{
			// Make sure that all alpha values are opaque.
			TArray<FColor>& RefBitmap = const_cast<TArray<FColor>&>(Bitmap);
			for (auto& Color : RefBitmap)
				Color.A = 255;

			TArray<uint8_t> last_compressed_png;
			FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
			last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
		});

		is_installed = true;
	}
}
