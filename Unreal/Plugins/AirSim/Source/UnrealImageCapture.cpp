#include "UnrealImageCapture.h"
#include "Engine/World.h"
#include "ImageUtils.h"

#include "RenderRequest.h"
#include "common/ClockFactory.hpp"
#include "CameraDirector.h"

UnrealImageCapture::UnrealImageCapture(ACameraDirector * camera_director, const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
    : cameras_(cameras)
    , camera_director_(camera_director)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
    nodisplay_ =  ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY == static_cast<ECameraDirectorMode>(::msr::airlib::AirSimSettings::singleton().initial_view_mode);
}

UnrealImageCapture::~UnrealImageCapture()
{}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, 
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    if (cameras_->valsSize() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses);
}


void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, 
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;

    bool visibilityChanged = false;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(requests.at(i).camera_name);
        //TODO: may be we should have these methods non-const?
        visibilityChanged = const_cast<UnrealImageCapture*>(this)->
            updateCameraVisibility(camera, requests[i]) || visibilityChanged;
    }
    if (!nodisplay_ && visibilityChanged) {
        // Wait for render so that view is ready for capture
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_->at(requests.at(i).camera_name);
        responses.push_back(ImageResponse());
        ImageResponse& response = responses.at(i);

        UTextureRenderTarget2D* textureTarget = nullptr;
        USceneCaptureComponent2D* capture = camera->getCaptureComponent(requests[i].image_type, false);
        if (capture == nullptr) {
            response.message = "Can't take screenshot because none camera type is not active";
        }
        else if (capture->TextureTarget == nullptr) {
            response.message = "Can't take screenshot because texture target is null";
        }
        else 
            textureTarget = capture->TextureTarget;

        render_params.push_back(std::make_shared<RenderRequest::RenderParams>(capture, textureTarget, requests[i].pixels_as_float, requests[i].compress));
    }

    RenderRequest render_request {};
    render_request.getScreenshot(render_params.data(), render_results, render_params.size(), camera_director_,
        [this, &requests](unsigned request_index)->msr::airlib::Pose {
            const ImageRequest& request = requests.at(request_index);
            APIPCamera* camera = cameras_->at(request.camera_name);
            return camera->getPose();
        });

    for (unsigned int i = 0; i < requests.size(); ++i) {
        const ImageRequest& request = requests.at(i);
        ImageResponse& response = responses.at(i);
              
        response.camera_name = request.camera_name;
        response.time_stamp = render_results[i]->time_stamp;
        response.image_data_uint8 = std::vector<uint8_t>(render_results[i]->image_data_uint8.GetData(), render_results[i]->image_data_uint8.GetData() + render_results[i]->image_data_uint8.Num());
        response.image_data_float = std::vector<float>(render_results[i]->image_data_float.GetData(), render_results[i]->image_data_float.GetData() + render_results[i]->image_data_float.Num());

        response.camera_position = render_results[i]->camera_pose.position;
        response.camera_orientation = render_results[i]->camera_pose.orientation;
        response.pixels_as_float = request.pixels_as_float;
        response.compress = request.compress;
        response.width = render_results[i]->width;
        response.height = render_results[i]->height;
        response.image_type = request.image_type;
    }

}


bool UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (! camera->getCameraTypeEnabled(request.image_type)) {
        camera->setCameraTypeEnabled(request.image_type, true);
        visibilityChanged = true;
    }

    return visibilityChanged;
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
            for(auto& Color : RefBitmap)
                Color.A = 255;

            TArray<uint8_t> last_compressed_png;
            FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
            last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
        });

        is_installed = true;
    }
}
