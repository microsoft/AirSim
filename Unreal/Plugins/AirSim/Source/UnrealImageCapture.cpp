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

    RenderRequest render_request(BufferPool_);
    render_request.fast_param_ = RenderRequest::RenderParams{ capture, textureTarget, false, false };
    render_request.FastScreenshot();

    int height = capture->TextureTarget->SizeY;
    int width = capture->TextureTarget->SizeX;
    int stride = render_request.latest_result_.stride;
    int bytes = render_request.latest_result_.pixels->size();
    int bytes_per_pixel = bytes / (height * width);
    int padded_width = stride / bytes_per_pixel;

    response.camera_name = camera_name;
    response.time_stamp = render_request.latest_result_.time_stamp;
    response.width = padded_width;
    response.height = height;
    response.image_type = image_type;
    response.image_data_uint8 = std::move(render_request.latest_result_.pixels);

    // Disable camera after capturing image, this reduces resource consumption when images are not being taken
    // Particulary when a high-resolution camera is used occasionally
    camera->setCameraTypeEnabled(image_type, false);
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
