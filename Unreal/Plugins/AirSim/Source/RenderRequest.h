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
    std::mutex mtx_;
    std::condition_variable cv_;
    bool fast_cap_done_ = false;

    FTextureRenderTargetResource* fast_rt_resource_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    //UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

    BufferPool<uint8_t>* buffer_pool_ = nullptr;
    BufferPool<float>* buffer_pool_float_ = nullptr;

public:
    RenderRequest(BufferPool<uint8_t> *buffer_pool, BufferPool<float> *buffer_pool_float);
    ~RenderRequest();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    void FastScreenshot();
    void RenderThreadScreenshotTask(RenderResult &result);

private:
    /**
     * \brief Converts BGRA8 data to BGR8 data.
     *
     * \param[out] dst     The image data to write to
     * \param[in]  src     The image data to convert
     * \param[in]  width   The width of the input image
     * \param[in]  height  The height of the input image
     * \param[in]  stride  The stride (bytes per line) of the input image
     */
    static void bgra8ToBgr8(
        uint8* dst,
        const uint8* src,
        uint32 width,
        uint32 height,
        uint32 stride);

    /**
     * \brief
     *     Extracts the red component from RGBA16F (AKA PF_FloatRGBA), converts
     *     it to float data, and then converts it from centimeters to meters
     *
     * \param[out] dst     The image data to write to
     * \param[in]  src     The image data to convert
     * \param[in]  width   The width of the input image
     * \param[in]  height  The height of the input image
     * \param[in]  stride  The stride (bytes per line) of the input image
     */
    static void rgba16fToDepthFloat(
        float* dst,
        const uint8* src,
        uint32 width,
        uint32 height,
        uint32 stride);
};
