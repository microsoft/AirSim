#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include "common/Common.hpp"


class RenderRequest : public FRenderCommand
{
public:
    struct RenderParams {
        USceneCaptureComponent2D *render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;
    };
    struct RenderResult {
        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        int width;
        int height;

        msr::airlib::TTimePoint time_stamp;
    };

private:
    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
public:
	RenderParams fast_param_{ nullptr, nullptr, false, false };
	std::vector<uint8_t> *rgba_output_ = nullptr;

private:
	volatile bool fast_cap_done_ = false;
	FTextureRenderTargetResource* fast_rt_resource_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    //UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

public:
    RenderRequest(std::vector<uint8_t>& rgb_output);
    ~RenderRequest();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

	void FastScreenshot();
	void RenderThreadScreenshotTask();
};
