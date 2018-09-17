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
#include "RandomPointPoseGeneratorNoRoll.h"
#include "../../SGM/src/sgmstereo/sgmstereo.h"
#include "../../SGM/src/stereoPipeline/StateStereo.h"
#include "writePNG.h"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

//NOTE: baseline (float B) and FOV (float fov) need to be set correctly!
class DataCollectorSGM {

private:
    //baseline * focal_length = depth * disparity
    float fov = Utils::degreesToRadians(90.0f);
    float B = 0.25; 
    float f = w / (2 * tan(fov/2));

public:
    DataCollectorSGM(std::string storage_dir)
        : storage_dir_(storage_dir)
    {
        FileSystem::ensureFolder(storage_dir);
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"left"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"right"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"depth_gt"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"disparity_gt"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"disparity_gt_viz"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"depth_sgm"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"disparity_sgm"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"disparity_sgm_viz"));
        FileSystem::ensureFolder(FileSystem::combine(storage_dir,"confidence_sgm"));
    }
    
    int generate(int num_samples)
    {
        msr::airlib::MultirotorRpcLibClient client;
        client.confirmConnection();
        client.reset();

        std::vector<ImageRequest> request = { 
            ImageRequest("front_left", ImageType::Scene, false, false), 
            ImageRequest("front_right", ImageType::Scene, false, false),
            ImageRequest("front_left", ImageType::DepthPlanner, true),
            ImageRequest("front_left", ImageType::DisparityNormalized, true)
        };
        const std::vector<ImageResponse>& response_init = client.simGetImages(request);
        w = response_init.at(0).width;
        h = response_init.at(0).height;

        msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
        RandomPointPoseGeneratorNoRoll pose_generator(static_cast<int>(clock->nowNanos()));
        std::fstream file_list(FileSystem::combine(storage_dir_, "files_list.txt"), 
            std::ios::out | std::ios::in | std::ios_base::app);

        int sample = getImageCount(file_list);
   
        p_state = new CStateStereo();
        p_state->Initialize(params,h,w);
        //Print SGM parameters
	    params.Print();

        try {
            while(sample < num_samples) {

                pose_generator.next();
                client.simSetVehiclePose(Pose(pose_generator.position, pose_generator.orientation), true);

                const auto& collision_info = client.simGetCollisionInfo();
                if (collision_info.has_collided) {
                    std::cout << "Collision at " << VectorMath::toString(collision_info.position) << std::endl;
                    continue;
                }
                ++sample;
                //Get into position
                clock->sleep_for(0.5);

                auto start_nanos = clock->nowNanos();

                const std::vector<ImageResponse>& response = client.simGetImages(request);
                if (response.size() != 4) {
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

                processImages(&result);

            }
        } catch (rpc::timeout &t) {
            // will display a message like
            // rpc::timeout: Timeout of 50ms while calling RPC function 'sleep'

            std::cout << t.what() << std::endl;
        }

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
    SGMOptions params;
    CStateStereo *p_state;
    //Image resolution
    int w;
    int h;
    float dtime = 0;

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

    void processImages(ImagesResult* result)
    {

            msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();


            auto process_time = clock->nowNanos();

            //Initialze file names
            std::string left_file_name = Utils::stringf("left/%06d.png", result->sample);
            std::string right_file_name = Utils::stringf("right/%06d.png", result->sample);
            std::string depth_gt_file_name  = Utils::stringf("depth_gt/%06d.pfm", result->sample);
            std::string disparity_gt_file_name  = Utils::stringf("disparity_gt/%06d.pfm", result->sample);
            std::string disparity_gt_viz_file_name  = Utils::stringf("disparity_gt_viz/%06d.png", result->sample);
            std::string depth_sgm_file_name  = Utils::stringf("depth_sgm/%06d.pfm", result->sample);
            std::string disparity_sgm_file_name  = Utils::stringf("disparity_sgm/%06d.pfm", result->sample);
            std::string disparity_sgm_viz_file_name  = Utils::stringf("disparity_sgm_viz/%06d.png", result->sample);
            std::string confidence_sgm_file_name  = Utils::stringf("confidence_sgm/%06d.png", result->sample);

            //Initialize data containers
            std::vector<uint8_t> left_img(h*w*3);
            std::vector<uint8_t> right_img(h*w*3);
            std::vector<float> gt_depth_data = result->response.at(2).image_data_float;
            std::vector<float> gt_disparity_data = result->response.at(3).image_data_float;
            std::vector<float> sgm_depth_data(h*w);
            std::vector<float> sgm_disparity_data(h*w);
            std::vector<uint8_t> sgm_confidence_data(h*w);

            //Remove alpha from RGB images
            int counter = 0;
            for (int idx = 0; idx < (h*w*4); idx++) {
                if ((idx+1) % 4 == 0) {
                    counter++; 
                    continue;
                }
                left_img[idx-counter] = result->response.at(0).image_data_uint8 [idx];
                right_img[idx-counter] = result->response.at(1).image_data_uint8 [idx];
            }

            //Get SGM disparity and confidence
            p_state->ProcessFrameAirSim(result->sample,dtime,left_img, right_img);

            //Get adjust SGM disparity and compute depth
	        for (int idx = 0; idx < (h*w); idx++)
	        {
		        float d = p_state->dispMap[idx];
		        if (d < FLT_MAX)
		        {
                    sgm_depth_data[idx] = -(B*f/d);
                    sgm_disparity_data[idx] = -d;
		        }
                sgm_confidence_data[idx] = p_state->confMap[idx];
                
	        }

            //Write files to disk
            //Left and right RGB image
            FILE *img_l = fopen(FileSystem::combine(result->storage_dir_, left_file_name).c_str(), "wb");
            svpng(img_l,w,h,reinterpret_cast<const unsigned char*>(left_img.data()),0);
            fclose(img_l);
            FILE *img_r = fopen(FileSystem::combine(result->storage_dir_, right_file_name).c_str(), "wb");
            svpng(img_r,w,h,reinterpret_cast<const unsigned char*>(right_img.data()),0);
            fclose(img_r);

            //GT disparity and depth
            Utils::writePfmFile(gt_depth_data.data(), w, h, FileSystem::combine(result->storage_dir_, depth_gt_file_name));
            denormalizeDisparity(gt_disparity_data, w);
            Utils::writePfmFile(gt_disparity_data.data(), w, h, FileSystem::combine(result->storage_dir_, disparity_gt_file_name));
            
            //SGM depth disparity and confidence
            Utils::writePfmFile(sgm_depth_data.data(), w, h, FileSystem::combine(result->storage_dir_, depth_sgm_file_name));
            Utils::writePfmFile(sgm_disparity_data.data(), w, h, FileSystem::combine(result->storage_dir_, disparity_sgm_file_name));
            FILE *sgm_c = fopen(FileSystem::combine(result->storage_dir_, confidence_sgm_file_name).c_str(), "wb");
            svpng(sgm_c,w,h,reinterpret_cast<const unsigned char*>(sgm_confidence_data.data()),0,1);
            fclose(sgm_c);

            //GT and SGM disparity for visulatization
            std::vector<uint8_t> sgm_disparity_viz(h*w*3);
            getColorVisualization(sgm_disparity_data, sgm_disparity_viz, h, w, 0.05f*w);
            FILE *disparity_sgm = fopen(FileSystem::combine(result->storage_dir_, disparity_sgm_viz_file_name).c_str(), "wb");
            svpng(disparity_sgm,w,h,reinterpret_cast<const unsigned char*>(sgm_disparity_viz.data()), 0);
            fclose(disparity_sgm);
            std::vector<uint8_t> gt_disparity_viz(h*w*3);
            getColorVisualization(gt_disparity_data, gt_disparity_viz, h, w, 0.05f*w);
            FILE *disparity_gt = fopen(FileSystem::combine(result->storage_dir_, disparity_gt_viz_file_name).c_str(), "wb");
            svpng(disparity_gt,w,h,reinterpret_cast<const unsigned char*>(gt_disparity_viz.data()), 0);
            fclose(disparity_gt);

            //Add all to file record
            (* result->file_list) << left_file_name << "," << right_file_name << "," << depth_gt_file_name << "," << disparity_gt_file_name << "," << depth_sgm_file_name << "," << disparity_sgm_file_name << "," << confidence_sgm_file_name << std::endl;

            std::cout << "Image #" << result->sample 
                << " pos:" << VectorMath::toString(result->position)
                << " ori:" << VectorMath::toString(result->orientation)
                << " render time " << result->render_time * 1E3f << "ms" 
                << " process time " << clock->elapsedSince(process_time) * 1E3f << " ms"
                << std::endl;

    }

    void getcolor(float d, float max_d, float&r, float&g, float&b) {
    
        float x = 6.0f;
        float inc = x / max_d;

        if (d < max_d)
            x = d * inc;
        else
            x = max_d * inc;

        r = 0.0f; g = 0.0f; b = 0.0f;
        if ((0 <= x && x <= 1) || (5 <= x && x < 6)) r = 1.0f;
        else if (4 <= x && x < 5) r = x - 4;
        else if (1 <= x && x < 2) r = 1.0f - (x - 1);

        if (1 <= x && x < 3) g = 1.0f;
        else if (0 <= x && x < 1) g = x - 0;
        else if (3 <= x && x < 4) g = 1.0f - (x - 3);

        if (3 <= x && x < 5) b = 1.0f;
        else if (2 <= x && x < 3) b = x - 2;
        else if (5 <= x && x < 6) b = 1.0f - (x - 5);
    }

    template <typename T> 
    void getColorVisualization(T& img, std::vector<uint8_t>& img_viz, int h, int w, float max_val = 255) {
	    for (int idx = 0; idx < (h*w); idx++)
	    {
		    float r,g,b;
            getcolor(img[idx],max_val,r,g,b);
            img_viz[idx*3] = (uint8_t) (r*255);
            img_viz[idx*3+1] = (uint8_t) (g*255);
            img_viz[idx*3+2] = (uint8_t) (b*255);
	    }    
    }

    template <typename T> 
    void getGrayVisualization(T& img, std::vector<uint8_t>& img_viz, int h, int w, int invert = 0, float scale = 1) {
        img_viz.resize(h*w);
	    for (int idx = 0; idx < (h*w); idx++)
	    {
            img_viz[idx] = (uint8_t) (invert ? scale/img[idx] : img[idx]*scale);  
        }
    }

    static void saveImageToFile(const std::vector<uint8_t>& image_data, const std::string& file_name)
    {
        std::ofstream file(file_name , std::ios::binary);
        file.write((char*) image_data.data(), image_data.size());
        file.close();
    }

    static void convertToPlanDepth(std::vector<float>& image_data, int width, int height, float f_px = 320)
    {
        float center_i = width / 2.0f - 1;
        float center_j = height / 2.0f - 1;

        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                float dist = std::sqrt((i - center_i)*(i - center_i) + (j - center_j)*(j - center_j));
                float denom = (dist / f_px);
                denom *= denom;
                denom = std::sqrt(1 + denom);
                image_data[j * width + i] /= denom;
            }
        }
    }

    static void convertToDisparity(std::vector<float>& image_data, float f_px = 320, float baseline_meters = 1)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = f_px * baseline_meters * (1.0f / image_data[i]);
        }
    }

    static void denormalizeDisparity(std::vector<float>& image_data, int width)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = image_data[i] * width;
        }
    }

};

