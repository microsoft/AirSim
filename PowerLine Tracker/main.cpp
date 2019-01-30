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
#include <string>
#include <fstream>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Windows.h>

using namespace cv;
msr::airlib::MultirotorRpcLibClient client;


cv::Mat getDroneImage() {
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	vector<ImageRequest> request = {
			ImageRequest("1", ImageType::Scene, false, false)
	};
	const vector<ImageResponse>& response = client.simGetImages(request);
	const uchar* im = response.at(0).image_data_uint8.data();
	cv::Mat img = cv::Mat(response[0].height, response[0].width, CV_8UC4, (void*)im);
	return img.clone();
}

void ShowImage(const string &name, const cv::Mat &img, int x = 50, int y = 50) {
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::moveWindow(name, x, y);
	cv::resizeWindow(name, 600, 500);
	cv::imshow(name, img);
}

void moveToStartPos() {
	msr::airlib::Vector3r pos(0.1, 0.1, -0.2);
	msr::airlib::Quaternionr ori= client.getMultirotorState().getOrientation();
	ori.w() = 0;
	ori.x() = 0;
	ori.y() = 0;
	ori.z() = 0;


	client.simSetVehiclePose(msr::airlib::Pose(pos,ori), false);
}

void print_pos() {
	auto position = client.getMultirotorState().getPosition();
	std::cout << "Position(" << position.x() << "," << position.y() << "," << position.z() << ")" << std::endl;
}

bool isPressed(std::string Letters) {
	for (size_t i = 0; i < Letters.size(); i++) {
		if (!(GetKeyState(Letters[i]) & 0x8000)) {
			return false;
		}
	}
	return true;
}

/*std::vector<float> keyboard_action(void) {
	float A[3] = { 0,-3,0 };
	float D[3] = { 0,3,0 };
	float W[3] = { 3,0,0 };
	float S[3] = { -3,0,0 };
	float Space[3] = { 0,0,-3 };
	float Shift[3] = { 0,0,3 };
	float* array[] = { A,W,S,D,Space,Shift };

	string shift({ VK_SHIFT,'\0' });
	string letters("AWSD ");
	letters += shift;
	std::vector<float> res = { 0,0,0 };
	for (size_t i = 0; i < letters.size(); i++) {
		std::cout << "Cheacking key: " << letters[i] << std::endl;
		if (GetKeyState(letters[i]) & 0x8000) {
			std::cout << "Key pressed: " << letters[i] << std::endl;
			res[0] += array[i][0];
			res[1] += array[i][1];
			res[2] += array[i][2];
		}
	}
	return res;
}*/

int main() 
{
    using namespace msr::airlib;

    
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    
    try {

		// ########### StartUp ################
        client.confirmConnection();
		std::cout << "Enable API Control" << std::endl;
		client.enableApiControl(true);
		//moveToStartPos();
		client.armDisarm(true);
		

		// ############ Take OFF ###############
		std::cout << "Take off" << std::endl;
		auto position = client.getMultirotorState().getPosition();
		auto orientation = client.getMultirotorState().getOrientation();
		std::cout << "Position(" << position.x() << "," << position.y() << "," << position.z() << ")" << std::endl;

		client.takeoffAsync(5)->waitOnLastTask();
		client.hoverAsync()->waitOnLastTask();

		// ############ Move Drone ###############
		std::cout << "MoveDrone" << std::endl;
		

		float speed = 6.0f;
		client.moveToPositionAsync(47.2094, -51.0386, -46.4161, speed)->waitOnLastTask();
		std::cout << "Hover and Rotate" << std::endl;
		client.hoverAsync()->rotateToYawAsync(2);


		// ############ Player Operated ##################
		std::cout << "Player Has control" << std::endl;
		cv::Mat image = getDroneImage();

		if (!image.empty()) {
			ShowImage("images", image);
			waitKey(1);
		}
		string shift({ VK_SHIFT,'\0' });
		string left({ VK_LEFT,'\0' });
		string right({ VK_RIGHT,'\0' });
		while (!isPressed("Q")){
			/*std::vector<float> res = keyboard_action();
			client.moveByVelocityAsync(res[0], res[1], res[2], 3);
			std::cout << "res: " << res[0] << ", " << res[1] << ", " << res[2] << std::endl;*/
		
			if (isPressed("WD")) {
				client.moveByVelocityAsync(3, 3, 0, 3);

			}else if (isPressed("AW")) {
				client.moveByVelocityAsync(3, -3, 0, 3);

			}else if (isPressed("DS")) {
				client.moveByVelocityAsync(-3, 3, 0, 3);

			}else if (isPressed("AS")) {
				client.moveByVelocityAsync(-3, -3, 0, 3);

			}else if (isPressed("A")){
				client.moveByVelocityAsync(0, -3, 0, 3);

			}else if (isPressed("D")){
				client.moveByVelocityAsync(0, 3, 0, 3);

			}else if(isPressed("W")){
				client.moveByVelocityAsync(3, 0, 0, 3);

			}else if(isPressed("S")){
				client.moveByVelocityAsync(-3, 0, 0, 3);

			}else if (isPressed(" ")) {
				client.moveByVelocityAsync(0, 0, -3, 3); //UP

			}else if (isPressed(shift)) {
				client.moveByVelocityAsync(0, 0, 3, 3); //down

			}else{
				client.moveByVelocityAsync(0, 0, 0, 3);
				client.hoverAsync();
			}
			//print_pos();

			/*image = getDroneImage();
			if (!image.empty()) {
				ShowImage("images", image);
				waitKey(1);
			}*/
		}


		// ########### TakeImage ############
		image = getDroneImage();

		if (!image.empty()) {
			ShowImage("images", image);
			waitKey(1);
		}
		
		position = client.getMultirotorState().getPosition();
		std::cout << "Position(" << position.x() << "," << position.y() << "," << position.z() << ")" << std::endl;


		// ################ End Program
		std::cout << "Press Enter to land" << std::endl; std::cin.get();
		moveToStartPos();
		client.armDisarm(false);

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}

