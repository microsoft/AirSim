// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <iostream>
#include <iomanip>
#include "common/Common.hpp"
#include "common/common_utils/ProsumerQueue.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/ClockFactory.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "RandomPointPoseGenerator.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON


class StereoImageGenerator {
public:
    StereoImageGenerator(std::string storage_dir)
        : storage_dir_(storage_dir)
    {
        FileSystem::ensureFolder(storage_dir);
    }

    int generate(int num_samples)
    {
        msr::airlib::MultirotorRpcLibClient client;
        client.confirmConnection();

        msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
        RandomPointPoseGenerator pose_generator(static_cast<int>(clock->nowNanos()));
        std::fstream file_list(FileSystem::combine(storage_dir_, "files_list.txt"), 
            std::ios::out | std::ios::in | std::ios_base::app);

        int sample = getImageCount(file_list);

        common_utils::ProsumerQueue<ImagesResult> results;
        std::thread result_process_thread(processImages, & results);

        try {
            while(sample < num_samples) {
                //const auto& collision_info = client.getCollisionInfo();
                //if (collision_info.has_collided) {
                //    pose_generator.next();
                //    client.simSetPose(pose_generator.position, pose_generator.orientation);

                //    std::cout << "Collision at " << VectorMath::toString(collision_info.position)
                //        << "Moving to next pose: "  << VectorMath::toString(pose_generator.position)
                //        << std::endl;

                //    continue;
                //}
                ++sample;

                auto start_nanos = clock->nowNanos();

                std::vector<ImageRequest> request = { 
                    ImageRequest("0", ImageType::Scene), 
                    ImageRequest("1", ImageType::Scene),
                    ImageRequest("1", ImageType::DisparityNormalized, true)
                };
                const std::vector<ImageResponse>& response = client.simGetImages(request);
                if (response.size() != 3) {
                    std::cout << "Images were not received!" << std::endl;
                    start_nanos = clock->nowNanos();
                    continue;
                }

                ImagesResult result;
                result.file_list = &file_list;
                result.response = response;
                result.sample = sample;
                result.render_time = clock->elapsedSince(start_nanos);;
                result.storage_dir_ = storage_dir_;
                result.position = pose_generator.position;
                result.orientation = pose_generator.orientation;


                results.push(result);

                pose_generator.next();
                client.simSetVehiclePose(Pose(pose_generator.position, pose_generator.orientation), true);
            }
        } catch (rpc::timeout &t) {
            // will display a message like
            // rpc::timeout: Timeout of 50ms while calling RPC function 'sleep'

            std::cout << t.what() << std::endl;
        }

        results.setIsDone(true);
        result_process_thread.join();
        return 0;
    }


private:
    typedef common_utils::FileSystem FileSystem;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::Pose Pose;
    typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
    typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    std::string storage_dir_;
    bool spawn_ue4 = false;
private:
    struct ImagesResult {
        std::vector<ImageResponse> response;
        msr::airlib::TTimeDelta render_time;
        std::string storage_dir_;
        std::fstream* file_list;
        int sample;
        Vector3r position;
        Quaternionr orientation;
    };


    static int getImageCount(std::fstream& file_list)
    {
        int sample = 0;
        std::string line;
        while (std::getline(file_list, line))
            ++sample;
        if (file_list.eof())
            file_list.clear();  //otherwise we can't do any further I/O
        else if (file_list.bad()) {
            throw  std::runtime_error("Error occurred while reading files_list.txt");
        }

        return sample;
    }

    static void processImages(common_utils::ProsumerQueue<ImagesResult>* results)
    {
        while (!results->getIsDone()) {
            msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();

            ImagesResult result;
            if (!results->tryPop(result)) {
                clock->sleep_for(1);
                continue;
            }

            auto process_time = clock->nowNanos();

            std::string left_file_name = Utils::stringf("left_%06d.png", result.sample);
            std::string right_file_name = Utils::stringf("right_%06d.png", result.sample);
            std::string disparity_file_name  = Utils::stringf("disparity_%06d.pfm", result.sample);
            saveImageToFile(result.response.at(0).image_data_uint8, 
                FileSystem::combine(result.storage_dir_, right_file_name));
            saveImageToFile(result.response.at(1).image_data_uint8, 
                FileSystem::combine(result.storage_dir_, left_file_name));

            std::vector<float> disparity_data = result.response.at(2).image_data_float;

            //writeFilePFM(depth_data, response.at(2).width, response.at(2).height,
            //    FileSystem::combine(storage_dir_, Utils::stringf("depth_%06d.pfm", i)));
            
            //below is not needed because we get disparity directly
            //convertToPlanDepth(depth_data, result.response.at(2).width, result.response.at(2).height);
            //float f = result.response.at(2).width / 2.0f - 1;
            //convertToDisparity(depth_data, result.response.at(2).width, result.response.at(2).height, f, 25 / 100.0f);

            denormalizeDisparity(disparity_data, result.response.at(2).width);

            Utils::writePfmFile(disparity_data.data(), result.response.at(2).width, result.response.at(2).height,
                FileSystem::combine(result.storage_dir_, disparity_file_name));

            (* result.file_list) << left_file_name << "," << right_file_name << "," << disparity_file_name << std::endl;

            std::cout << "Image #" << result.sample 
                << " pos:" << VectorMath::toString(result.position)
                << " ori:" << VectorMath::toString(result.orientation)
                << " render time " << result.render_time * 1E3f << "ms" 
                << " process time " << clock->elapsedSince(process_time) * 1E3f << " ms"
                << std::endl;

        }
    }

    static void saveImageToFile(const std::vector<uint8_t>& image_data, const std::string& file_name)
    {
        std::ofstream file(file_name , std::ios::binary);
        file.write((char*) image_data.data(), image_data.size());
        file.close();
    }

    static void convertToPlanDepth(std::vector<float>& image_data, int width, int height, float f = 320)
    {
        float center_i = width / 2.0f - 1;
        float center_j = height / 2.0f - 1;

        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                float dist = std::sqrt((i - center_i)*(i - center_i) + (j - center_j)*(j - center_j));
                float denom = (dist / f);
                denom *= denom;
                denom = std::sqrt(1 + denom);
                image_data[j * width + i] /= denom;
            }
        }
    }

    static void convertToDisparity(std::vector<float>& image_data, int width, int height, float f = 320, float baseline_meters = 1)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = f * baseline_meters * (1.0f / image_data[i]);
        }
    }

    static void denormalizeDisparity(std::vector<float>& image_data, int width)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = image_data[i] * width;
        }
    }

};

