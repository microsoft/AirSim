#include "UnrealImageCapture.h"
#include "RenderRequest.h"
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"



UnrealImageCapture::UnrealImageCapture(const std::vector<APIPCamera*>& cameras) 
    : cameras_(cameras)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, 
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses)
{
    if (cameras_.size() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses, false);
}


void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests, 
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method)
{
    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;

    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = cameras_[requests.at(i).camera_id];
        responses.push_back(ImageResponse());
        ImageResponse& response = responses.at(i);


        updateCameraVisibility(camera, requests[i]);

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

        render_params.push_back(std::make_shared<RenderRequest::RenderParams>(textureTarget, requests[i].pixels_as_float, requests[i].compress));
    }

    RenderRequest render_request(use_safe_method);
    render_request.getScreenshot(render_params.data(), render_results, render_params.size());

    for (unsigned int i = 0; i < requests.size(); ++i) {
        const ImageRequest& request = requests.at(i);
        ImageResponse& response = responses.at(i);
        APIPCamera* camera = cameras_[request.camera_id];
              
        response.camera_id = request.camera_id;
        response.time_stamp = render_results[i]->time_stamp;
        response.image_data_uint8 = std::vector<uint8_t>(render_results[i]->image_data_uint8.GetData(), render_results[i]->image_data_uint8.GetData() + render_results[i]->image_data_uint8.Num());
        response.image_data_float = std::vector<float>(render_results[i]->image_data_float.GetData(), render_results[i]->image_data_float.GetData() + render_results[i]->image_data_float.Num());

        msr::airlib::Pose pose = camera->getPose();
        response.camera_position = pose.position;
        response.camera_orientation = pose.orientation;
        response.pixels_as_float = request.pixels_as_float;
        response.compress = request.compress;
        response.width = render_results[i]->width;
        response.height = render_results[i]->height;
        response.image_type = request.image_type;
    }

}


void UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (! camera->getCameraTypeEnabled(request.image_type)) {
        camera->setCameraTypeEnabled(request.image_type, true);
        visibilityChanged = true;
    }

    if (visibilityChanged) {
        // Wait for render so that view is ready for capture
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
        // not sure why this doesn't work.
        //DECLARE_CYCLE_STAT(TEXT("FNullGraphTask.CheckRenderStatus"), STAT_FNullGraphTask_CheckRenderStatus, STATGROUP_TaskGraphTasks);
        //auto renderStatus = TGraphTask<FNullGraphTask>::CreateTask(NULL).ConstructAndDispatchWhenReady(GET_STATID(STAT_FNullGraphTask_CheckRenderStatus), ENamedThreads::RenderThread);
        //FTaskGraphInterface::Get().WaitUntilTaskCompletes(renderStatus);
    }
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
