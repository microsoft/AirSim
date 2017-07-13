#include "AirSim.h"
#include "RenderRequest.h"
#include "ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "NedTransform.h"


VehicleCameraConnector::VehicleCameraConnector(APIPCamera* camera) 
    : camera_(camera)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

msr::airlib::VehicleCameraBase::ImageResponse VehicleCameraConnector::getImage(VehicleCameraConnector::ImageType image_type, bool pixels_as_float, bool compress)
{

    if (camera_== nullptr) {
        ImageResponse response;
        response.message = "camera is not set";
        return response;
    }
    if (image_type == ImageType_::None) {
        ImageResponse response;
        response.message = "ImageType was None";
        return response;
    }

    return getSceneCaptureImage(image_type, pixels_as_float, compress, false);
}

msr::airlib::VehicleCameraBase::ImageResponse VehicleCameraConnector::getSceneCaptureImage(VehicleCameraConnector::ImageType image_type, 
    bool pixels_as_float, bool compress, bool use_safe_method)
{
    bool visibilityChanged = false;
    if ((camera_->getEnableCameraTypes() & image_type) == ImageType_::None
        && (image_type != ImageType_::None)) {
        camera_->setEnableCameraTypes(camera_->getEnableCameraTypes() | image_type);
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

    using namespace msr::airlib;
    USceneCaptureComponent2D* capture = camera_->getCaptureComponent(image_type, false);
    if (capture == nullptr) {
        ImageResponse response;
        response.message = "Can't take screenshot because none camera type is not active";
        return response;
    }

    if (capture->TextureTarget == nullptr) {
        ImageResponse response;
        response.message = "Can't take screenshot because texture target is null";
        return response;
    }

    UTextureRenderTarget2D* textureTarget = capture->TextureTarget;

    RenderRequest request(use_safe_method);
    int width, height;
    TArray<uint8> image;
    request.getScreenshot(textureTarget, image, pixels_as_float, compress, width, height);

    ImageResponse response;
    response.time_stamp = msr::airlib::ClockFactory::get()->nowNanos();
    response.image_data = std::vector<uint8_t>(image.GetData(), image.GetData() + image.Num());
    response.camera_position = NedTransform::toNedMeters(capture->GetComponentLocation());
    response.camera_orientation = NedTransform::toQuaternionr(capture->GetComponentRotation().Quaternion(), true);
    response.pixels_as_float = pixels_as_float;
    response.compress = compress;
    response.width = width;
    response.height = height;

    return response;
}


bool VehicleCameraConnector::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
    FScreenshotRequest::RequestScreenshot(false); // This is an async operation
    return true;
}

void VehicleCameraConnector::addScreenCaptureHandler(UWorld *world)
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
