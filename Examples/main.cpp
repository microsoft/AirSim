#include "StandAloneSensors.hpp"
#include "StandAlonePhysics.hpp"
#include "StereoImageGenerator.hpp"
#include "GaussianMarkovTest.hpp"
#include "DepthNav.hpp"
#include <iostream>
#include <string>


int runStandAloneSensors(int argc, const char *argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <out_file_name> <period_ms> <total_duration_sec>" << std::endl;
        return 1;
    }

    float period = 30E-3f;
    if (argc >= 3)
        period = std::stof(argv[2]) * 1E-3f;

    float total_duration = 3600;
    if (argc >= 4)
        total_duration = std::stof(argv[3]);

    std::cout << "Period is " << period << "sec" << std::endl;
    std::cout << "Total duration is " << total_duration << "sec" << std::endl;


    using namespace msr::airlib;

    //60 acres park:
    //GeoPoint testLocation(47.7037051477, -122.1415384809, 9.93f); 

    //marymoore park
    //GeoPoint testLocation(47.662804385, -122.1167039875, 9.93f);

    GeoPoint testLocation(47.7631699747, -122.0685655406, 9.93f); // woodinville
    float yawOffset = 0;// static_cast<float>(91.27622  * M_PI / 180.0); // I was aligned with the road...

    std::ofstream out_file(argv[1]);
    StandALoneSensors::generateImuStaticData(out_file, period, total_duration);
    StandALoneSensors::generateBarometerStaticData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateBarometerDynamicData(out_file, period, total_duration, testLocation);
    StandALoneSensors::generateMagnetometer2D(out_file, period, total_duration, testLocation, yawOffset, true);
    StandALoneSensors::generateMagnetometerMap(out_file);

    return 0;
}

int runStandAlonePhysics(int argc, const char *argv[])
{
    using namespace msr::airlib;

    StandAlonePhysics::testCollison();

    return 0;
}


void runSteroImageGenerator(int num_samples, std::string storage_path)

{

	StereoImageGenerator gen(storage_path);

	gen.generate(num_samples);

}



void runSteroImageGenerator(int argc, const char *argv[])

{

	runSteroImageGenerator(argc < 2 ? 50000 : std::stoi(argv[1]), argc < 3 ?

		common_utils::FileSystem::combine(

			common_utils::FileSystem::getAppDataFolder(), "stereo_gen")

		: std::string(argv[2]));

}


int main(int argc, const char *argv[])
{
    using namespace msr::airlib;

	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	typedef common_utils::FileSystem FileSystem;

	MultirotorRpcLibClient client;

	//GaussianMarkovTest test;
	//test.run();
	DepthNav depthNav;
	Vector3r x = Vector3r(1, 2, 3);
	Vector3r y = Vector3r(2, 3, 4);
	//Size of UAV
	Vector2r uav_size = Vector2r(0.29 * 3, 0.98 * 2); //height:0.29 x width : 0.98 - allow some tolerance
													  //Define start and goal poses
	Pose startPose = Pose(Vector3r(0, 5, -1), Quaternionr(0, 0, 0, 0)); //start pose
	Pose goalPose = Pose(Vector3r(120, 0, -1), Quaternionr(0, 0, 0, 0)); //final pose

	Quaternionr quat;
	quat = depthNav.getQuatFromVecs(x, y);
	float vfov = depthNav.hfov2vfov(Utils::degreesToRadians(90.0f), Vector2r(480, 640));

	std::cout << "Quaternion: " << quat.w() << quat.x() << quat.y() << quat.z() << std::endl;
	std::cout << Utils::radiansToDegrees(vfov) << std::endl;
	std::cout << std::endl;

	try {
		client.confirmConnection();
		client.simSetPose(startPose, true);

		bool bGoalReached = false;

		while (bGoalReached != true) {

			std::vector<ImageRequest> request = {
				ImageRequest(1, ImageType::DepthPlanner, true),
				ImageRequest(1, ImageType::Scene),
				ImageRequest(1, ImageType::DisparityNormalized, true)
			};

			const std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() == 0) {
				std::cout << "No images recieved!" << std::endl;
				continue;
			}
			else {
				std::cout << "# of images recieved: " << response.size() << std::endl;
			}

			for (const ImageResponse& image_info : response) {
				if (image_info.image_type == ImageType::DepthPlanner)
				{
					if (image_info.image_data_float.size() > 0) 
					{
						std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
						Vector2r image_sz = Vector2r(image_info.height, image_info.width);
						Vector2r bb_sz = depthNav.compute_bb(image_sz, uav_size, Utils::degreesToRadians(90.0f), 5.f);

						//compute box of interest
						std::vector<float> crop;
						float min_depth = 1000.f;

						for (int i = int((image_sz.x() - bb_sz.x()) / 2); i < int((image_sz.x() + bb_sz.x()) / 2); i++) {
							for (int j = int((image_sz.y() - bb_sz.y()) / 2); j<int((image_sz.y() + bb_sz.y()) / 2); j++) {
								int idx = i * int(image_sz.y()) + j;
								crop.push_back(image_info.image_data_float.data()[idx]);
								std::cout << idx << "  " << image_info.image_data_float.data()[idx] << std::endl;
								if (image_info.image_data_float.data()[idx] < min_depth) {
									min_depth = image_info.image_data_float.data()[idx];
								}
							}
						}
						std::cout <<  std::endl;
					}
					else 
					{
						std::cout << "No image data. Make sure pixels_as_float_val is set to true. " << std::endl;
					}

				}

			}

			bGoalReached = true;

		}
	}
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
	}


	

	
}

