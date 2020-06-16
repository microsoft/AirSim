#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include "common/Common.hpp"

class BufferPool; 

class RenderRequest : public FRenderCommand
{
public:
    struct RenderParams
    {
        USceneCaptureComponent2D *render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;
    };

    struct RenderResult {
        RenderResult() = default;
        // RenderResult(RenderResult&) = default;
        RenderResult(RenderResult&&) = default;
        RenderResult &operator=(RenderResult &&) = default;

        int width;
        int height;
        int stride;
        msr::airlib::TTimePoint time_stamp;
        bool pixels_as_float;
        std::unique_ptr<std::vector<uint8_t>, std::function<void(std::vector<uint8_t>*)>> pixels = nullptr;
        std::unique_ptr<std::vector<float>, std::function<void(std::vector<float>*)>> pixels_float = nullptr;
    };

private:
    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
public:
    RenderParams fast_param_{ nullptr, nullptr, false, false };
    RenderResult latest_result_{};

private:
    volatile bool fast_cap_done_ = false;
    FTextureRenderTargetResource* fast_rt_resource_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    //UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

    BufferPool* buffer_pool_ = nullptr;
public:
    RenderRequest(BufferPool *buffer_pool);
    ~RenderRequest();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    void FastScreenshot();
    void RenderThreadScreenshotTask(RenderResult &result);
};
