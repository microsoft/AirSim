// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <iostream>
#include <iomanip>
#include "common/Common.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/ClockFactory.hpp"
#include "rpc/RpcLibClient.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "PoseGenerators.hpp"
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

        client.confirmConnection();
    }

    int generate(int num_samples)
    {
        msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
        RandomPointPoseGenerator pose_generator(& client, static_cast<int>(clock->nowNanos()));
        std::fstream file_list(FileSystem::combine(storage_dir_, "files_list.txt"), 
            std::ios::out | std::ios::in | std::ios_base::app);

        int sample = 0;
        std::string line;
        while (std::getline(file_list, line))
            ++sample;
        if (file_list.eof())
            file_list.clear();  //otherwise we can't do any further I/O
        else if (file_list.bad()) {
            std::cout << "Error occured while reading files_list.txt";
            return 1;
        }

        while(sample < num_samples) {
            const auto& collision_info = client.getCollisionInfo();
            if (collision_info.has_collided) {
                std::cout << "Collison at " << VectorMath::toString(collision_info.position)
                    << "Moving to next pose." << std::endl;

                pose_generator.next();
                continue;
            }
            ++sample;

            auto start_nanos = clock->nowNanos();

            std::vector<ImageRequest> request = { 
                ImageRequest(0, ImageType_::Scene), 
                ImageRequest(1, ImageType_::Scene),
                ImageRequest(1, ImageType_::Depth, true)
            };
            const std::vector<ImageResponse>& response = client.simGetImages(request);
            if (response.size() != 3) {
                std::cout << "Images were not recieved!" << std::endl;
                start_nanos = clock->nowNanos();
                continue;
            }

            auto render_time = clock->elapsedSince(start_nanos);

            std::string left_file_name = Utils::stringf("left_%06d.png", sample);
            std::string right_file_name = Utils::stringf("right_%06d.png", sample);
            std::string disparity_file_name  = Utils::stringf("disparity_%06d.pfm", sample);
            saveImageToFile(response.at(0).image_data, 
                FileSystem::combine(storage_dir_, right_file_name));
            saveImageToFile(response.at(1).image_data, 
                FileSystem::combine(storage_dir_, left_file_name));

            std::vector<float> depth_data;
            const auto& depth_raw = response.at(2).image_data;
            for (int k = 0; k < depth_raw.size(); k += sizeof(float)) {
                float pixel_float = *(reinterpret_cast<const float*>(&depth_raw[k]));
                depth_data.push_back(pixel_float);
            }

            //writeFilePFM(depth_data, response.at(2).width, response.at(2).height,
            //    FileSystem::combine(storage_dir_, Utils::stringf("depth_%06d.pfm", i)));

            convertToPlanDepth(depth_data, response.at(2).width, response.at(2).height);

            float f = response.at(2).width / 2.0f - 1;
            convertToDisparity(depth_data, response.at(2).width, response.at(2).height, f, 25 / 100.0f);
            writeFilePFM(depth_data, response.at(2).width, response.at(2).height,
                FileSystem::combine(storage_dir_, disparity_file_name));

            file_list << left_file_name << "," << right_file_name << "," << disparity_file_name << std::endl;

            std::cout << "Image #" << sample 
                << " done in " << (clock->elapsedSince(start_nanos)) * 1E3f << "ms" 
                << " render time " << render_time * 1E3f << " ms"
                << std::endl;

            pose_generator.next();
        }

        return 0;
    }

private:
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

    // check whether machine is little endian
    static int littleendian()
    {
        int intval = 1;
        unsigned char *uval = (unsigned char *)&intval;
        return uval[0] == 1;
    }

    static void writeFilePFM(const std::vector<float>& image_data, int width, int height, std::string path, float scalef=1)
    {
        std::fstream file(path.c_str(), std::ios::out | std::ios::binary);

        std::string bands;
        float fvalue;       // scale factor and temp value to hold pixel value
        bands = "Pf";   // grayscale

        // sign of scalefact indicates endianness, see pfms specs
        if(littleendian())
            scalef = -scalef;

        // insert header information 
        file << bands   << "\n";
        file << width   << "\n";
        file << height  << "\n";
        file << scalef  << "\n";

        if(bands == "Pf"){          // handle 1-band image 
            for (int i=0; i < height; i++) {
                for(int j=0; j < width; ++j){
                    fvalue = image_data.at(i * width + j);
                    file.write((char*) &fvalue, sizeof(fvalue));
                }
            }
        }
    }

private:
    typedef common_utils::FileSystem FileSystem;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef msr::airlib::DroneControllerBase::ImageRequest ImageRequest;
    typedef msr::airlib::VehicleCameraBase::ImageResponse ImageResponse;
    typedef msr::airlib::VehicleCameraBase::ImageType_ ImageType_;

    std::string storage_dir_;
    msr::airlib::RpcLibClient client;
};

