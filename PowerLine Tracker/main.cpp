// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "PLineD.hpp"

#include <iostream>
#include <fstream>
#include <chrono>

#include <opencv2/opencv.hpp>

using namespace cv;

cv::Mat getDroneImage(msr::airlib::MultirotorRpcLibClient &client) {
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	vector<ImageRequest> request = {
			ImageRequest("1", ImageType::Scene, false, false)
	};
	const vector<ImageResponse>& response = client.simGetImages(request);
	const uchar* im = response.at(0).image_data_uint8.data();
	cv::Mat img = cv::Mat(response[0].height, response[0].width, CV_8UC4, (void*)im);
	return img;
}


int main() 
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    
    try {
        client.confirmConnection();

		/*vector<ImageRequest> request = {
			ImageRequest("1", ImageType::Scene, false, false)
		};
        const vector<ImageResponse>& response = client.simGetImages(request);
		const uchar* im = response.at(0).image_data_uint8.data();
		Mat img2 = cv::Mat(response[0].height, response[0].width,CV_8UC4, (void*)im);
		//Mat img = imdecode(response.at(0).image_data_uint8.data(), cv::IMREAD_COLOR);
		std::cout << "Response size: "<< response[0].width << "x" << response[0].height << std::endl;*/

		cv::Mat image = getDroneImage(client);

		if (!image.empty()) {

			imshow("img", image);
			waitKey(0);
		}
        

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}
