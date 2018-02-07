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
#include "vehicles/multirotor/controllers/DroneControllerBase.hpp"
#include "RandomPointPoseGenerator.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#define REQUEST_JUST_LEFT 1
#define REQUEST_JUST_DEPTH 2
#define REQUEST_LEFT_AND_TRUTH 3

#define REQUEST_TYPE REQUEST_LEFT_AND_TRUTH

#define SAVE_FILES 1


class ImagesWithTruthGenerator {
public:
    ImagesWithTruthGenerator(std::string storage_dir)
        : storage_dir_(storage_dir)
    {
        FileSystem::ensureFolder(storage_dir);
    }

    int generate(int num_samples)
    {
        msr::airlib::MultirotorRpcLibClient client;
        client.confirmConnection();

        msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
        //RandomPointPoseGenerator pose_generator(static_cast<int>(clock->nowNanos()));
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

                //    std::cout << "Collison at " << VectorMath::toString(collision_info.position)
                //        << "Moving to next pose: "  << VectorMath::toString(pose_generator.position)
                //        << std::endl;

                //    continue;
                //}
                ++sample;

                auto start_nanos = clock->nowNanos();

				
                std::vector<ImageRequest> request = { 
#if REQUEST_TYPE == REQUEST_JUST_LEFT
					ImageRequest(0, ImageType::Scene, false, (SAVE_FILES==1))
#elif REQUEST_TYPE == REQUEST_JUST_DEPTH
					ImageRequest(0, ImageType::DepthPlanner, true, false)
#elif REQUEST_TYPE == REQUEST_LEFT_AND_TRUTH
					ImageRequest(0, ImageType::Scene, false, (SAVE_FILES==1)), 
					ImageRequest(1, ImageType::DepthPlanner, true, false)
#endif					
                };

				msr::airlib::Pose myPose = client.simGetPose();
				msr::airlib::Vector3r myVel = client.getVelocity();
				//msr::airlib::RCData myRcData = client.getRCData();
				//msr::airlib::GeoPoint myGeoPoint = client.getGpsLocation();

                const std::vector<ImageResponse>& response = client.simGetImages(request);
				
				int correct_response_size = 1;
#if REQUEST_TYPE == REQUEST_LEFT_AND_TRUTH
				correct_response_size = 2;
#endif
				
                if (response.size() != correct_response_size) {
                    std::cout << "Images were not received!" << std::endl;
                    start_nanos = clock->nowNanos();
                    continue;
                }

                ImagesResult result;
                result.file_list = &file_list;
                result.response = response;
                result.sample = sample;
                result.render_time = clock->elapsedSince(start_nanos);
                result.storage_dir_ = storage_dir_;
				result.position = myPose.position;
				result.orientation = myPose.orientation;
				result.velocity = myVel;
				//result.rcData = myRcData;
				//result.geoPoint = myGeoPoint;				

                results.push(result);

                //pose_generator.next();
                //client.simSetPose(Pose(pose_generator.position, pose_generator.orientation), true);
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
	using RCData = msr::airlib::RCData;
	using GeoPoint = msr::airlib::GeoPoint;

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
		Vector3r velocity;
		RCData rcData;
		GeoPoint geoPoint;
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
            throw  std::runtime_error("Error occured while reading files_list.txt");
        }

        return sample;
    }

    static void processImages(common_utils::ProsumerQueue<ImagesResult>* results)
    {
		static int response_cnt = 0;
        while (!results->getIsDone()) {
            msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
			
			response_cnt++;

            ImagesResult result;
            if (!results->tryPop(result)) {
                clock->sleep_for(1);
                continue;
            }

            auto process_time = clock->nowNanos();
			
#if REQUEST_TYPE == REQUEST_JUST_LEFT  || REQUEST_TYPE == REQUEST_LEFT_AND_TRUTH
			std::string left_file_name = Utils::stringf("left_%06d.png", result.sample);
#endif
#if REQUEST_TYPE == REQUEST_JUST_DEPTH || REQUEST_TYPE == REQUEST_LEFT_AND_TRUTH
			std::string depth_file_name  = Utils::stringf("depth_%06d.pfm", result.sample);
#endif
	
			//static int last_count = response_cnt;
			
			//msr::airlib::ClockBase* clock = msr::airlib::ClockFactory::get();
			
			static msr::airlib::TTimePoint lastAvgTime = clock->nowNanos();
			static int last_response_cnt = response_cnt;
		
#if REQUEST_TYPE == REQUEST_JUST_LEFT
	#if SAVE_FILES
			saveImageToFile(result.response.at(0).image_data_uint8, 
					FileSystem::combine(result.storage_dir_, left_file_name));
			(*result.file_list) << left_file_name
				<< "," << VectorMath::toString(result.position)
				<< "," << VectorMath::toString(result.orientation)
				<< "," << VectorMath::toString(result.velocity)
				//<< "," << result.geoPoint.to_string()
				<< std::endl;
	#endif	
	
			//auto start_nanos = clock->nowNanos();
			//result.render_time = clock->elapsedSince(start_nanos);;
			//msr::airlib::TTimeDelta render_time;
		
#elif REQUEST_TYPE == REQUEST_JUST_DEPTH
	#if SAVE_FILES
			std::vector<float> depth_data = result.response.at(0).image_data_float;
			Utils::writePfmFile(depth_data.data(), result.response.at(0).width, result.response.at(0).height,   
				FileSystem::combine(result.storage_dir_, depth_file_name));	
			(* result.file_list) << depth_file_name 
				<< "," << VectorMath::toString(result.position)
				<< "," << VectorMath::toString(result.orientation)
				<< "," << VectorMath::toString(result.velocity)
				//<< "," << result.geoPoint.to_string()
				<< std::endl;
	#endif

#elif REQUEST_TYPE == REQUEST_LEFT_AND_TRUTH
	#if SAVE_FILES
			saveImageToFile(result.response.at(0).image_data_uint8, 
				FileSystem::combine(result.storage_dir_, left_file_name));
			std::vector<float> depth_data = result.response.at(1).image_data_float;
			Utils::writePfmFile(depth_data.data(), result.response.at(1).width, result.response.at(1).height,   
				FileSystem::combine(result.storage_dir_, depth_file_name));	
			(* result.file_list) << left_file_name << "," << depth_file_name 
				<< "," << VectorMath::toString(result.position)
				<< "," << VectorMath::toString(result.orientation)
				<< "," << VectorMath::toString(result.velocity)
				//<< "," << result.geoPoint.to_string()
				<< std::endl;
	#endif
#endif
			if (response_cnt % 10 == 0) {			
				msr::airlib::TTimeDelta dt = clock->elapsedSince(lastAvgTime);
				lastAvgTime = clock->nowNanos();
				
				float frameRate = ((float)(response_cnt - last_response_cnt))/((float)dt);
				last_response_cnt = response_cnt;			
				
				std::cout << "Image #" << result.sample
					<< " count: " << response_cnt
					<< " Frame Rate: " << frameRate
					<< " dt: " << dt
					//<< " VEL: " << VectorMath::toString(result.velocity)
					//<< " pos:" << VectorMath::toString(result.position)
					//<< " ori:" << VectorMath::toString(result.orientation)
					<< " render time " << result.render_time * 1E3f << "ms" 
					<< " process time " << clock->elapsedSince(process_time) * 1E3f << " ms"
					<< std::endl;
			}

        }
    }

    static void saveImageToFile(const std::vector<uint8_t>& image_data, const std::string& file_name)
    {
        std::ofstream file(file_name , std::ios::binary);
        file.write((char*) image_data.data(), image_data.size());
        file.close();
    }

    /*static void convertToPlanDepth(std::vector<float>& image_data, int width, int height, float f = 320)
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
    }*/

    /*static void convertToDisparity(std::vector<float>& image_data, int width, int height, float f = 320, float baseline_meters = 1)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = f * baseline_meters * (1.0f / image_data[i]);
        }
    }*/

	/*
    static void denormalizeDisparity(std::vector<float>& image_data, int width)
    {
        for (int i = 0; i < image_data.size(); ++i) {
            image_data[i] = image_data[i] * width;
        }
    }*/

};

