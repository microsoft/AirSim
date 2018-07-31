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

void testDepthNav()
{
	DepthNav depthNav;
	//Test getCellCenters
	std::vector<Vector2r> cell_centers = depthNav.getCellCenters();
	std::vector<float> img(256 * 144, 5.5f);
	img[2] = 0.5f;
	img[3] = 0.5f;
	img[4] = 0.5f;
	std::cout << depthNav.isCellFree(img, cell_centers[0]) << std::endl;
	std::cout << depthNav.isCellFree(img, cell_centers[1]) << std::endl;

	//Test planeSize
	Vector2r planeSize = depthNav.getPlaneSize(1, Utils::degreesToRadians(90.0f), depthNav.hfov2vfov(Utils::degreesToRadians(90.0f), 144, 256));
	//x_min, y_min, x_max, y_max for this plane in body frame.
	float z_min = -planeSize.x() / 2;
	float y_min = -planeSize.y() / 2;
	float z_max = planeSize.x() / 2;
	float y_max = planeSize.y() / 2;
	std::cout << "Plane Boundary: " << "z_min:" << z_min << " z_max:" << z_max << " y_min:" << y_min << " y_max:" << y_max << std::endl;
	
	//Test planeIntersection, nearest neighbor, isCellFree and spiralOrder.
	Vector3r goal_vec = Vector3r(5, -5, 0);
	Vector3r forward_vec = VectorMath::transformToWorldFrame(VectorMath::front(), Quaternionr(1, 0, 0, 0));
	Vector3r intersect_point = depthNav.linePlaneIntersection(goal_vec, forward_vec, 1);
	float y_px = intersect_point.y() * 256 / planeSize.y() + 256 / 2;
	float z_px = intersect_point.z() * 144 / planeSize.x() + 144 / 2;
	std::cout << y_px << ", " << z_px << std::endl;
	unsigned int cell_idx = depthNav.nearest_neighbor(cell_centers, Vector2r(y_px, z_px));
	std::cout << cell_centers[cell_idx].x() << ", " << cell_centers[cell_idx].y() << std::endl;

	std::cout << depthNav.isCellFree(img, cell_centers[cell_idx]) << std::endl;

	std::vector<int> spiral_idxs = depthNav.spiralOrder(12, 14, cell_idx);
}



int main(int argc, const char *argv[])
{
	DepthNav depthNav;

	using namespace msr::airlib;

	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	typedef common_utils::FileSystem FileSystem;

	MultirotorRpcLibClient client;

	Pose startPose = Pose(Vector3r(0, 0, -1), Quaternionr(1, 0, 0, 0)); //start pose
	Pose currentPose;
	//Pose goalPose = Pose(Vector3r(25, 10, -1), Quaternionr(1, 0, 0, 0)); //final pose
	Pose goalPose = Pose(Vector3r(-25, 0, -1), Quaternionr(1, 0, 0, 0)); //final pose

	try {
		client.confirmConnection();
		client.reset();
		client.simSetVehiclePose(startPose, true);
		currentPose = startPose;

		bool bGoalReached = false;

		while (bGoalReached != true) {

			std::vector<ImageRequest> request = {
				ImageRequest("1", ImageType::DepthPlanner, true),
				ImageRequest("1", ImageType::Scene),
				ImageRequest("1", ImageType::DisparityNormalized, true)
			};

			const std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() == 0) {
				std::cout << "No images recieved!" << std::endl;
				continue;
			}
			else {
				//std::cout << "# of images recieved: " << response.size() << std::endl;
			}

			for (const ImageResponse& image_info : response) {
				if (image_info.image_type == ImageType::DepthPlanner)
				{
					if (image_info.image_data_float.size() > 0)
					{
						std::vector<float> img;

						for (int i=0; i<image_info.image_data_float.size();i++){ img.push_back(image_info.image_data_float.data()[i]); }
						
						currentPose = depthNav.getNextPose(img, goalPose.position, currentPose, 0.1f);
					    std::cout << "Position: " << currentPose.position << " Orientation: " << currentPose.orientation  << std::endl;

						client.simSetVehiclePose(currentPose, true);

						float dist2goal = depthNav.getDistanceToGoal(currentPose.position, goalPose.position);
						
						if (dist2goal < 1) {
							std::cout << "Target reached." << std::endl; std::cin.get();
							return 0;
						}
						else {
							std::cout << "Distance to target: " << dist2goal << std::endl;
						}
						

					}
				}
			}
		}
	}

	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
		//Add some sleep
		std::this_thread::sleep_for(std::chrono::duration<double>(5));
	}

	return 0;
}





/*
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
	DepthNavTest depthNavTest;

	//Size of UAV
	Vector2r uav_size = Vector2r(0.29 * 3, 0.98 * 2); //height:0.29 x width : 0.98 - allow some tolerance
	
	float threshold = 5.f;

    //Define start and goal poses
	Pose startPose = Pose(Vector3r(5, 0, -1), Quaternionr(1, 0, 0, 0)); //start pose
	Pose currentPose;
	//Pose goalPose = Pose(Vector3r(-50, 4, -15), Quaternionr(1, 0, 0, 0)); //final pose
	//Pose goalPose = Pose(Vector3r(50, -12, -1), Quaternionr(1, 0, 0, 0)); //final pose
	Pose goalPose = Pose(Vector3r(50, 4, -1), Quaternionr(1, 0, 0, 0)); //final pose

	Quaternionr currentQuat;
	Quaternionr nextQuat;
	Quaternionr goalQuat;
	Vector3r forwardVec = VectorMath::front();
	Vector3r goalVec;
	float hfov = Utils::degreesToRadians(90.0f);
	float step = 0.1f;
	bool bSafeToMove = true;

	try {
		client.confirmConnection();
		client.reset();
		client.simSetVehiclePose(startPose, true);
		currentPose = startPose;

		bool bGoalReached = false;

		while (bGoalReached != true) {

			std::vector<ImageRequest> request = {
				ImageRequest("1", ImageType::DepthPlanner, true),
				ImageRequest("1", ImageType::Scene),
				ImageRequest("1", ImageType::DisparityNormalized, true)
			};

			const std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() == 0) {
				std::cout << "No images recieved!" << std::endl;
				continue;
			}
			else {
				std::cout << "# of images recieved: " << response.size() << std::endl;
			}

			float min_depth = 1000.f;


			for (const ImageResponse& image_info : response) {
				if (image_info.image_type == ImageType::DepthPlanner)
				{
					if (image_info.image_data_float.size() > 0)
					{

						std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
						Vector2r image_sz = Vector2r(image_info.height, image_info.width);
						Vector2r bb_sz = depthNavTest.compute_bb_sz(image_sz, uav_size, hfov, 5.f);

						//compute box of interest
						std::vector<float> crop;

						for (int i = int((image_sz.x() - bb_sz.x()) / 2); i < int((image_sz.x() + bb_sz.x()) / 2); i++) {
							for (int j = int((image_sz.y() - bb_sz.y()) / 2); j<int((image_sz.y() + bb_sz.y()) / 2); j++) {
								int idx = i * int(image_sz.y()) + j;
								crop.push_back(image_info.image_data_float.data()[idx]);
								//std::cout << idx << "  " << image_info.image_data_float.data()[idx] << std::endl;
								if (image_info.image_data_float.data()[idx] < min_depth) {
									min_depth = image_info.image_data_float.data()[idx];
								}
							}
						}
					}

					else
					{
						std::cout << "No image data. Make sure pixels_as_float_val is set to true. " << std::endl;
					}
				}
			}

				goalVec = goalPose.position - currentPose.position;
				goalQuat = VectorMath::lookAt(currentPose.position, goalPose.position);
				//goalQuat = depthNavTest.getQuatBetweenVecs(forwardVec, goalVec);

				Vector3r contact;
				bool linePlaneIntersection = depthNavTest.linePlaneIntersection(contact, goalVec, currentPose.position, VectorMath::transformToWorldFrame(forwardVec, currentPose.orientation), VectorMath::transformToWorldFrame(forwardVec, currentPose.orientation)*threshold + currentPose.position);
				Vector2r planeSz = depthNavTest.getPlaneSize(threshold, hfov, depthNavTest.hfov2vfov(hfov, Vector2r(144, 256)));
				planeSz = depthNavTest.getPlaneSize(threshold, hfov, depthNavTest.hfov2vfov(hfov, Vector2r(144, 256)));
				float z_min, z_max, y_min, y_max;
				depthNavTest.getPlaneBoundary(planeSz, VectorMath::transformToWorldFrame(forwardVec, currentPose.orientation)*threshold + currentPose.position,z_min,z_max,y_min,y_max);
				std::cout << "Plane Intersection: " << linePlaneIntersection << " at " << contact << std::endl;
				std::cout << "Plane Size: " << planeSz << std::endl;
				std::cout << "Plane Boundary: " << "z_min:" << z_min << " z_max:" << z_max << " y_min:" << y_min << " y_max:" << y_max << std::endl;

				if (min_depth < threshold)
				{
					//Turn to avoid obstacle
					currentPose.orientation = VectorMath::coordOrientationAdd(currentPose.orientation, VectorMath::toQuaternion(0, 0, Utils::degreesToRadians(3.f)));
					bSafeToMove = true;
				}
				else
				{
					//Turn towards goal
					currentPose.orientation = VectorMath::slerp(currentPose.orientation, goalQuat, 0.1f);
					bSafeToMove = true;
					//std::cout << "Quaternion: " << goalQuat.w() << " " << goalQuat.x() << " " << goalQuat.y() << " " << goalQuat.z() << std::endl;
				}

				//// To do rather than interpolate take physical constraints into account
				//real_T p_current, r_current, y_current;
				//VectorMath::toEulerianAngle(currentPose.orientation, p_current, r_current, y_current);

				//real_T p_goal, r_goal, y_goal;
				//VectorMath::toEulerianAngle(goalQuat, p_goal, r_goal, y_goal);

				////UAV rate in rad/s
				//real_T p_rate = Utils::degreesToRadians(45.0f);
				//real_T r_rate = Utils::degreesToRadians(45.0f);
				//real_T y_rate = Utils::degreesToRadians(90.0f);

				//real_T p_diff, r_diff, y_diff;
				//p_diff = p_goal - p_current;
				//r_diff = r_goal - r_current;
				//y_diff = y_goal - y_current;

				//real_T dt = 1 / 30;

				//real_T p_feasible, r_feasible, y_feasible;
				//p_feasible = abs(p_diff) > abs(p_rate*dt) ? copysignf(p_rate, p_diff) * dt : p_diff;
				//r_feasible = abs(r_diff) > abs(r_rate*dt) ? copysignf(r_rate, r_diff) * dt : r_diff;
				//y_feasible = abs(y_diff) > abs(y_rate*dt) ? copysignf(y_rate, y_diff) * dt : y_diff;

				//nextQuat = VectorMath::toQuaternion(p_feasible, r_feasible, y_feasible);
				//currentPose.orientation = nextQuat * currentPose.orientation;
				//

				
				if (bSafeToMove) 
				{
					currentPose.position = currentPose.position + VectorMath::transformToWorldFrame(forwardVec, currentPose.orientation) * step;
					//currentPose.position = currentPose.position + goalVec.normalized() * step;
				}

				client.simSetVehiclePose(currentPose, true);

				std::cout << "Distance to target: " << depthNavTest.getNorm2(goalVec) << std::endl;

				if (depthNavTest.getNorm2(goalVec) < 1) {
					std::cout << "Target reached." << std::endl; std::cin.get();
					return 0;
				}

		}
	}
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
		//Add some sleep
		std::this_thread::sleep_for(std::chrono::duration<double>(5));
	}

	
}

*/