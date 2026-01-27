#include <airsim_ros_wrapper.h>
#include "common/AirSimSettings.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std::placeholders;

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" }, // rgb
    { 1, "DepthPlanar" }, // depth
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" } // stereo infrared
};

// Helper function to map AirSim ImageType to RealSense stream name
std::string AirsimROSWrapper::get_realsense_stream_name(ImageType image_type, int camera_index) const
{
    switch (image_type) {
    case ImageType::Scene:
        return "color";
    case ImageType::DepthPlanar:
        return "depth";
    case ImageType::Infrared:
        // First IR camera is infra1, second is infra2
        return (camera_index == 0) ? "infra1" : "infra2";
    default:
        // For other types, use the original mapping
        return image_type_int_to_string_map_.at(static_cast<int>(image_type));
    }
}

// Helper function to get RealSense frame ID from ImageType
std::string AirsimROSWrapper::get_realsense_frame_id(ImageType image_type, int camera_index) const
{
    std::string stream_name = get_realsense_stream_name(image_type, camera_index);
    return "camera_" + stream_name + "_optical_frame";
}

AirsimROSWrapper::AirsimROSWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::string& host_ip)
    : airsim_settings_parser_(host_ip)
    , host_ip_(host_ip)
    , airsim_client_(nullptr)
    , airsim_client_lidar_(host_ip)
    , airsim_client_imu_(host_ip)
    , nh_(nh)
    , isENU_(false)
    , publish_clock_(false)
    , unite_imu_method_(0)
{
    ros_clock_.clock = rclcpp::Time(0);

    if (AirSimSettings::singleton().simmode_name != AirSimSettings::kSimModeTypeCar) {
        airsim_mode_ = AIRSIM_MODE::DRONE;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to DRONE mode");
    }
    else {
        airsim_mode_ = AIRSIM_MODE::CAR;
        RCLCPP_INFO(nh_->get_logger(), "Setting ROS wrapper to CAR mode");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

    initialize_ros();

    RCLCPP_INFO(nh_->get_logger(), "AirsimROSWrapper Initialized!");
}

void AirsimROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try {

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::MultirotorRpcLibClient(host_ip_));
        }
        else {
            airsim_client_ = std::unique_ptr<msr::airlib::RpcLibClientBase>(new msr::airlib::CarRpcLibClient(host_ip_));
        }
        airsim_client_->confirmConnection();
        // Confirm connections for all parallel image clients
        for (auto& img_client : airsim_client_images_vec_) {
            img_client->confirmConnection();
        }
        airsim_client_lidar_.confirmConnection();
        airsim_client_imu_.confirmConnection();

        // Enable API control and arm vehicles with retry logic
        // Vehicles may not be spawned immediately, so we retry a few times
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            const std::string& vehicle_name = vehicle_name_ptr_pair.first;
            bool api_control_enabled = false;
            bool vehicle_armed = false;
            const int max_retries = 10;
            const double retry_delay_sec = 1.0;

            // Retry enabling API control
            for (int retry = 0; retry < max_retries && !api_control_enabled; ++retry) {
                try {
                    airsim_client_->enableApiControl(true, vehicle_name);
                    api_control_enabled = true;
                    RCLCPP_INFO(nh_->get_logger(), "API control enabled for vehicle: %s", vehicle_name.c_str());
                }
                catch (rpc::rpc_error& e) {
                    std::string msg = e.get_error().as<std::string>();
                    if (retry < max_retries - 1) {
                        RCLCPP_WARN(nh_->get_logger(), "Failed to enable API control for vehicle %s (attempt %d/%d): %s. Retrying...", vehicle_name.c_str(), retry + 1, max_retries, msg.c_str());
                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(retry_delay_sec * 1000)));
                    }
                    else {
                        RCLCPP_ERROR(nh_->get_logger(), "Failed to enable API control for vehicle %s after %d attempts: %s", vehicle_name.c_str(), max_retries, msg.c_str());
                    }
                }
            }

            // Retry arming vehicle (only if API control was enabled)
            if (api_control_enabled) {
                for (int retry = 0; retry < max_retries && !vehicle_armed; ++retry) {
                    try {
                        airsim_client_->armDisarm(true, vehicle_name);
                        vehicle_armed = true;
                        RCLCPP_INFO(nh_->get_logger(), "Vehicle armed: %s", vehicle_name.c_str());
                    }
                    catch (rpc::rpc_error& e) {
                        std::string msg = e.get_error().as<std::string>();
                        if (retry < max_retries - 1) {
                            RCLCPP_WARN(nh_->get_logger(), "Failed to arm vehicle %s (attempt %d/%d): %s. Retrying...", vehicle_name.c_str(), retry + 1, max_retries, msg.c_str());
                            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(retry_delay_sec * 1000)));
                        }
                        else {
                            RCLCPP_WARN(nh_->get_logger(), "Failed to arm vehicle %s after %d attempts: %s. Continuing anyway...", vehicle_name.c_str(), max_retries, msg.c_str());
                        }
                    }
                }
            }
        }

        // Get origin geo point (may fail if no vehicles are spawned yet)
        try {
            origin_geo_point_ = get_origin_geo_point();
            origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
            RCLCPP_INFO(nh_->get_logger(), "Origin geo point retrieved successfully");
        }
        catch (rpc::rpc_error& e) {
            RCLCPP_WARN(nh_->get_logger(), "Could not retrieve origin geo point (vehicles may not be spawned yet): %s", e.get_error().as<std::string>().c_str());
            // Set default values
            origin_geo_point_ = msr::airlib::GeoPoint();
            origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API during initialization: %s", msg.c_str());
        RCLCPP_WARN(nh_->get_logger(), "Some operations may have failed. The node will continue, but some features may not work until vehicles are spawned.");
        // Don't shutdown - allow the node to continue and retry operations later
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Standard exception during AirSim initialization: %s", e.what());
        RCLCPP_WARN(nh_->get_logger(), "The node will continue, but some features may not work.");
    }
}

void AirsimROSWrapper::initialize_ros()
{

    // ros params
    double update_airsim_control_every_n_sec;
    nh_->get_parameter("is_vulkan", is_vulkan_);
    nh_->get_parameter("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    nh_->get_parameter("publish_clock", publish_clock_);
    nh_->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
    odom_frame_id_ = world_frame_id_ == AIRSIM_FRAME_ID ? AIRSIM_ODOM_FRAME_ID : ENU_ODOM_FRAME_ID;
    nh_->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
    nh_->get_parameter_or("base_link_frame_id", base_link_frame_id_, base_link_frame_id_);
    nh_->get_parameter_or("camera_link_frame_id", camera_link_frame_id_, camera_link_frame_id_);
    isENU_ = (odom_frame_id_ == ENU_ODOM_FRAME_ID);
    nh_->get_parameter_or("coordinate_system_enu", isENU_, isENU_);
    nh_->get_parameter_or("unite_imu_method", unite_imu_method_, 0); // 0=none, 1=copy, 2=linear_interpolation
    vel_cmd_duration_ = 0.05; // todo rosparam
    // todo enforce dynamics constraints in this node as well?
    // nh_->get_parameter("max_vert_vel_", max_vert_vel_);
    // nh_->get_parameter("max_horz_vel", max_horz_vel_)

    nh_->declare_parameter("vehicle_name", rclcpp::ParameterValue(""));
    create_ros_pubs_from_settings_json();
    airsim_control_update_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(update_airsim_control_every_n_sec), std::bind(&AirsimROSWrapper::drone_state_timer_cb, this));
}

void AirsimROSWrapper::create_ros_pubs_from_settings_json()
{
    // subscribe to control commands on global nodehandle
    gimbal_angle_quat_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleQuatCmd>("~/gimbal_angle_quat_cmd", 50, std::bind(&AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this, _1));
    gimbal_angle_euler_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>("~/gimbal_angle_euler_cmd", 50, std::bind(&AirsimROSWrapper::gimbal_angle_euler_cmd_cb, this, _1));
    origin_geo_point_pub_ = nh_->create_publisher<airsim_interfaces::msg::GPSYaw>("~/origin_geo_point", 10);

    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    vehicle_name_ptr_map_.clear();
    size_t lidar_cnt = 0;

    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;

        nh_->set_parameter(rclcpp::Parameter("vehicle_name", curr_vehicle_name));

        set_nans_to_zeros_in_pose(*vehicle_setting);

        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            vehicle_ros = std::unique_ptr<MultiRotorROS>(new MultiRotorROS());
        }
        else {
            vehicle_ros = std::unique_ptr<CarROS>(new CarROS());
        }

        vehicle_ros->odom_frame_id_ = odom_frame_id_; // No vehicle prefix to match standard ROS convention
        vehicle_ros->vehicle_name_ = curr_vehicle_name;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);
        append_static_base_link_to_camera_link_tf(vehicle_ros.get());

        const std::string topic_prefix = "~/" + curr_vehicle_name;
        vehicle_ros->odom_local_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id_, 10);

        vehicle_ros->env_pub_ = nh_->create_publisher<airsim_interfaces::msg::Environment>(topic_prefix + "/environment", 10);

        vehicle_ros->global_gps_pub_ = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix + "/global_gps", 10);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

            // bind to a single callback. todo optimal subs queue length
            // bind multiple topics to a single callback, but keep track of which vehicle name it was by passing curr_vehicle_name as the 2nd argument

            std::function<void(const airsim_interfaces::msg::VelCmd::SharedPtr)> fcn_vel_cmd_body_frame_sub = std::bind(&AirsimROSWrapper::vel_cmd_body_frame_cb, this, _1, vehicle_ros->vehicle_name_);
            drone->vel_cmd_body_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_body_frame", 1, fcn_vel_cmd_body_frame_sub); // todo ros::TransportHints().tcpNoDelay();

            std::function<void(const airsim_interfaces::msg::VelCmd::SharedPtr)> fcn_vel_cmd_world_frame_sub = std::bind(&AirsimROSWrapper::vel_cmd_world_frame_cb, this, _1, vehicle_ros->vehicle_name_);
            drone->vel_cmd_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_world_frame", 1, fcn_vel_cmd_world_frame_sub);

            std::function<bool(std::shared_ptr<airsim_interfaces::srv::Takeoff::Request>, std::shared_ptr<airsim_interfaces::srv::Takeoff::Response>)> fcn_takeoff_srvr = std::bind(&AirsimROSWrapper::takeoff_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            drone->takeoff_srvr_ = nh_->create_service<airsim_interfaces::srv::Takeoff>(topic_prefix + "/takeoff", fcn_takeoff_srvr);

            std::function<bool(std::shared_ptr<airsim_interfaces::srv::Land::Request>, std::shared_ptr<airsim_interfaces::srv::Land::Response>)> fcn_land_srvr = std::bind(&AirsimROSWrapper::land_srv_cb, this, _1, _2, vehicle_ros->vehicle_name_);
            drone->land_srvr_ = nh_->create_service<airsim_interfaces::srv::Land>(topic_prefix + "/land", fcn_land_srvr);

            // vehicle_ros.reset_srvr = nh_->create_service(curr_vehicle_name + "/reset",&AirsimROSWrapper::reset_srv_cb, this);
        }
        else {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            std::function<void(const airsim_interfaces::msg::CarControls::SharedPtr)> fcn_car_cmd_sub = std::bind(&AirsimROSWrapper::car_cmd_cb, this, _1, vehicle_ros->vehicle_name_);
            car->car_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::CarControls>(topic_prefix + "/car_cmd", 1, fcn_car_cmd_sub);
            car->car_state_pub_ = nh_->create_publisher<airsim_interfaces::msg::CarState>(topic_prefix + "/car_state", 10);
        }

        // iterate over camera map std::map<std::string, CameraSetting> .cameras;
        int infra_camera_index = 0; // Track IR camera index for infra1/infra2 naming
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
            // Add RealSense-style camera sub-frames (infra1, infra2, depth, color)
            append_static_camera_subframes_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
            // camera_setting.gimbal
            std::vector<ImageRequest> current_image_request_vec;
            current_image_request_vec.clear();

            // iterate over capture_setting std::map<int, CaptureSetting> capture_settings
            for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                auto& capture_setting = curr_capture_elem.second;

                // todo why does AirSimSettings::loadCaptureSettings calls AirSimSettings::initializeCaptureSettings()
                // which initializes default capture settings for _all_ NINE msr::airlib::ImageCaptureBase::ImageType
                if (!(std::isnan(capture_setting.fov_degrees))) {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);
                    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
                    if (curr_image_type == ImageType::Scene || curr_image_type == ImageType::Segmentation || curr_image_type == ImageType::SurfaceNormals || curr_image_type == ImageType::Infrared) {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                    }
                    // if {DepthPlanar, DepthPerspective,DepthVis, DisparityNormalized}, get float image
                    else {
                        current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                    }

                    // Determine stream name: if camera name is infra1 or infra2, use it directly
                    // Otherwise, use ImageType-based mapping
                    std::string stream_name;
                    if (curr_camera_name == "infra1" || curr_camera_name == "infra2") {
                        // Keep infra1/infra2 naming regardless of ImageType
                        stream_name = curr_camera_name;
                    }
                    else {
                        // Use RealSense naming convention based on ImageType
                        stream_name = get_realsense_stream_name(curr_image_type,
                                                                (curr_image_type == ImageType::Infrared) ? infra_camera_index++ : 0);
                    }

                    const std::string camera_topic = "~/camera/" + stream_name + "/image_" +
                                                     ((curr_image_type == ImageType::DepthPlanar) ? "rect_" : "") + "raw";

                    // Use raw publisher for all images (no compression)
                    image_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::Image>(camera_topic, 1));
                    cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera/" + stream_name + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(stream_name, camera_setting, capture_setting));
                }
            }
            // push back pair (vector of image captures, current vehicle name)
            airsim_img_request_vehicle_name_pair_vec_.push_back(std::make_pair(current_image_request_vec, curr_vehicle_name));
        }

        // iterate over sensors
        for (auto& curr_sensor_map : vehicle_setting->sensors) {
            auto& sensor_name = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            if (sensor_setting->enabled) {

                switch (sensor_setting->sensor_type) {
                case SensorBase::SensorType::Barometer: {
                    SensorPublisher<airsim_interfaces::msg::Altimeter> sensor_publisher =
                        create_sensor_publisher<airsim_interfaces::msg::Altimeter>("Barometer", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/altimeter/" + sensor_name, 10);
                    vehicle_ros->barometer_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Imu: {
                    // Create separate gyro and accel publishers (RealSense-style)
                    SensorPublisher<sensor_msgs::msg::Imu> gyro_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Imu>("Gyro", sensor_setting->sensor_name, sensor_setting->sensor_type, "camera/gyro/sample", 10);
                    vehicle_ros->gyro_pubs_.emplace_back(gyro_publisher);

                    SensorPublisher<sensor_msgs::msg::Imu> accel_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Imu>("Accel", sensor_setting->sensor_name, sensor_setting->sensor_type, "camera/accel/sample", 10);
                    vehicle_ros->accel_pubs_.emplace_back(accel_publisher);

                    // Also keep legacy unified IMU publisher for backward compatibility
                    SensorPublisher<sensor_msgs::msg::Imu> imu_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Imu>("Imu", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/imu/" + sensor_name, 10);
                    vehicle_ros->imu_pubs_.emplace_back(imu_publisher);
                    break;
                }
                case SensorBase::SensorType::Gps: {
                    SensorPublisher<sensor_msgs::msg::NavSatFix> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::NavSatFix>("Gps", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gps/" + sensor_name, 10);
                    vehicle_ros->gps_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Magnetometer: {
                    SensorPublisher<sensor_msgs::msg::MagneticField> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::MagneticField>("Magnetometer", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/magnetometer/" + sensor_name, 10);
                    vehicle_ros->magnetometer_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Distance: {
                    SensorPublisher<sensor_msgs::msg::Range> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Range>("Distance", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/distance/" + sensor_name, 10);
                    vehicle_ros->distance_pubs_.emplace_back(sensor_publisher);
                    break;
                }
                case SensorBase::SensorType::Lidar: {
                    auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                    msr::airlib::LidarSimpleParams params;
                    params.initializeFromSettings(lidar_setting);
                    append_static_lidar_tf(vehicle_ros.get(), sensor_name, params);

                    SensorPublisher<sensor_msgs::msg::PointCloud2> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Lidar", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/lidar/" + sensor_name, 10);
                    vehicle_ros->lidar_pubs_.emplace_back(sensor_publisher);
                    lidar_cnt += 1;
                    break;
                }
                default: {
                    throw std::invalid_argument("Unexpected sensor type");
                }
                }
            }
        }

        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros)); // allows fast lookup in command callbacks in case of a lot of drones
    }

    // add takeoff and land all services if more than 2 drones
    if (vehicle_name_ptr_map_.size() > 1 && airsim_mode_ == AIRSIM_MODE::DRONE) {
        takeoff_all_srvr_ = nh_->create_service<airsim_interfaces::srv::Takeoff>("~/all_robots/takeoff", std::bind(&AirsimROSWrapper::takeoff_all_srv_cb, this, _1, _2));
        land_all_srvr_ = nh_->create_service<airsim_interfaces::srv::Land>("~/all_robots/land", std::bind(&AirsimROSWrapper::land_all_srv_cb, this, _1, _2));

        vel_cmd_all_body_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_all_body_frame_cb, this, _1));
        vel_cmd_all_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_all_world_frame_cb, this, _1));

        vel_cmd_group_body_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_group_body_frame_cb, this, _1));
        vel_cmd_group_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapper::vel_cmd_group_world_frame_cb, this, _1));

        takeoff_group_srvr_ = nh_->create_service<airsim_interfaces::srv::TakeoffGroup>("~/group_of_robots/takeoff", std::bind(&AirsimROSWrapper::takeoff_group_srv_cb, this, _1, _2));
        land_group_srvr_ = nh_->create_service<airsim_interfaces::srv::LandGroup>("~/group_of_robots/land", std::bind(&AirsimROSWrapper::land_group_srv_cb, this, _1, _2));
    }

    // todo add per vehicle reset in AirLib API
    reset_srvr_ = nh_->create_service<airsim_interfaces::srv::Reset>("~/reset", std::bind(&AirsimROSWrapper::reset_srv_cb, this, _1, _2));

    if (publish_clock_) {
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("~/clock", 1);
    }

    // Image timer at 30Hz with dedicated callback group and parallel fetching
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        constexpr double DEFAULT_IMG_PERIOD = 1.0 / 30.0; // 30Hz default
        double update_airsim_img_response_every_n_sec = DEFAULT_IMG_PERIOD;
        nh_->get_parameter_or("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec, DEFAULT_IMG_PERIOD);

        // Create one RPC client per camera for parallel image fetching
        size_t num_cameras = airsim_img_request_vehicle_name_pair_vec_.size();
        airsim_client_images_vec_.reserve(num_cameras);
        for (size_t i = 0; i < num_cameras; ++i) {
            airsim_client_images_vec_.push_back(
                std::make_unique<msr::airlib::RpcLibClientBase>(host_ip_));
        }
        RCLCPP_INFO(nh_->get_logger(), "Created %zu parallel image RPC clients for %zu cameras", num_cameras, num_cameras);

        img_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_img_response_timer_ = nh_->create_wall_timer(
            std::chrono::duration<double>(update_airsim_img_response_every_n_sec),
            std::bind(&AirsimROSWrapper::img_response_timer_cb, this),
            img_callback_group_);
        RCLCPP_INFO(nh_->get_logger(), "Image timer configured at %.1fHz (%.4fs period)", 1.0 / update_airsim_img_response_every_n_sec, update_airsim_img_response_every_n_sec);
    }

    // Lidar timer with dedicated callback group
    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec = 0.01; // 100Hz default
        nh_->get_parameter_or("update_lidar_every_n_sec", update_lidar_every_n_sec, 0.01);

        lidar_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_lidar_update_timer_ = nh_->create_wall_timer(
            std::chrono::duration<double>(update_lidar_every_n_sec),
            std::bind(&AirsimROSWrapper::lidar_timer_cb, this),
            lidar_callback_group_);
    }

    // IMU timer at 200Hz with dedicated callback group and RPC client
    size_t imu_cnt = 0;
    size_t gyro_cnt = 0;
    size_t accel_cnt = 0;
    for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        imu_cnt += vehicle_name_ptr_pair.second->imu_pubs_.size();
        gyro_cnt += vehicle_name_ptr_pair.second->gyro_pubs_.size();
        accel_cnt += vehicle_name_ptr_pair.second->accel_pubs_.size();
    }
    if (imu_cnt > 0 || gyro_cnt > 0 || accel_cnt > 0) {
        constexpr double DEFAULT_IMU_PERIOD = 1.0 / 200.0; // 200Hz default
        double update_imu_every_n_sec = DEFAULT_IMU_PERIOD;
        nh_->get_parameter_or("update_imu_every_n_sec", update_imu_every_n_sec, DEFAULT_IMU_PERIOD);

        // Create unified IMU publisher if unite_imu_method > 0
        if (unite_imu_method_ > 0 && (gyro_cnt > 0 || accel_cnt > 0)) {
            unified_imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>("~/camera/imu", 10);
            RCLCPP_INFO(nh_->get_logger(), "Unified IMU publisher created with method %d", unite_imu_method_);
        }

        imu_callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_imu_update_timer_ = nh_->create_wall_timer(
            std::chrono::duration<double>(update_imu_every_n_sec),
            std::bind(&AirsimROSWrapper::imu_timer_cb, this),
            imu_callback_group_);
        RCLCPP_INFO(nh_->get_logger(), "IMU timer configured at %.0fHz (%.4fs period)", 1.0 / update_imu_every_n_sec, update_imu_every_n_sec);
    }

    initialize_airsim();
}

// QoS - The depth of the publisher message queue.
// more details here - https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
template <typename T>
const SensorPublisher<T> AirsimROSWrapper::create_sensor_publisher(const std::string& sensor_type_name, const std::string& sensor_name,
                                                                   SensorBase::SensorType sensor_type, const std::string& topic_name, int QoS)
{
    RCLCPP_INFO_STREAM(nh_->get_logger(), sensor_type_name);
    SensorPublisher<T> sensor_publisher;
    sensor_publisher.sensor_name = sensor_name;
    sensor_publisher.sensor_type = sensor_type;
    sensor_publisher.publisher = nh_->create_publisher<T>("~/" + topic_name, QoS);
    return sensor_publisher;
}

// todo: error check. if state is not landed, return error.
bool AirsimROSWrapper::takeoff_srv_cb(std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request, std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response, const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask(); // todo value for timeout_sec?
    // response->success =
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);
    // response->success =

    return true;
}

bool AirsimROSWrapper::takeoff_group_srv_cb(std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Request> request, std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name)->waitOnLastTask(); // todo value for timeout_sec?
    // response->success =
    else
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name);
    // response->success =

    return true;
}

bool AirsimROSWrapper::takeoff_all_srv_cb(std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request, std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first)->waitOnLastTask(); // todo value for timeout_sec?
    // response->success =
    else
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->takeoffAsync(20, vehicle_name_ptr_pair.first);
    // response->success =

    return true;
}

bool AirsimROSWrapper::land_srv_cb(std::shared_ptr<airsim_interfaces::srv::Land::Request> request, std::shared_ptr<airsim_interfaces::srv::Land::Response> response, const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true; //todo
}

bool AirsimROSWrapper::land_group_srv_cb(std::shared_ptr<airsim_interfaces::srv::LandGroup::Request> request, std::shared_ptr<airsim_interfaces::srv::LandGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        for (const auto& vehicle_name : request->vehicle_names)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name);

    return true; //todo
}

bool AirsimROSWrapper::land_all_srv_cb(std::shared_ptr<airsim_interfaces::srv::Land::Request> request, std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    if (request->wait_on_last_task)
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first)->waitOnLastTask();
    else
        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_)
            static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->landAsync(60, vehicle_name_ptr_pair.first);

    return true; //todo
}

// todo add reset by vehicle_name API to airlib
// todo not async remove wait_on_last_task
bool AirsimROSWrapper::reset_srv_cb(std::shared_ptr<airsim_interfaces::srv::Reset::Request> request, std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
{
    unused(request);
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);

    airsim_client_->reset();
    return true; //todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::msg::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z);
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z());
}

void AirsimROSWrapper::car_cmd_cb(const airsim_interfaces::msg::CarControls::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto car = static_cast<CarROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    car->car_cmd_.throttle = msg->throttle;
    car->car_cmd_.steering = msg->steering;
    car->car_cmd_.brake = msg->brake;
    car->car_cmd_.handbrake = msg->handbrake;
    car->car_cmd_.is_manual_gear = msg->manual;
    car->car_cmd_.manual_gear = msg->manual_gear;
    car->car_cmd_.gear_immediate = msg->gear_immediate;

    car->has_car_cmd_ = true;
}

msr::airlib::Pose AirsimROSWrapper::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& airlib_quat) const
{
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), airlib_quat);
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_ = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
    drone->has_vel_cmd_ = true;
}

void AirsimROSWrapper::vel_cmd_group_body_frame_cb(const airsim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (const auto& vehicle_name : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
        drone->vel_cmd_ = get_airlib_body_vel_cmd(msg->vel_cmd, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_all_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    // todo expose wait_on_last_task or nah?
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());
        drone->vel_cmd_ = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_ = get_airlib_world_vel_cmd(*msg);
    drone->has_vel_cmd_ = true;
}

// this is kinda unnecessary but maybe it makes life easier for the end user.
void AirsimROSWrapper::vel_cmd_group_world_frame_cb(const airsim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    for (const auto& vehicle_name : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
        drone->vel_cmd_ = get_airlib_world_vel_cmd(msg->vel_cmd);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapper::vel_cmd_all_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);

    // todo expose wait_on_last_task or nah?
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_pair.second.get());
        drone->vel_cmd_ = get_airlib_world_vel_cmd(*msg);
        drone->has_vel_cmd_ = true;
    }
}

// todo support multiple gimbal commands
void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(const airsim_interfaces::msg::GimbalAngleQuatCmd::SharedPtr gimbal_angle_quat_cmd_msg)
{
    tf2::Quaternion quat_control_cmd;
    try {
        tf2::convert(gimbal_angle_quat_cmd_msg->orientation, quat_control_cmd);
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd); // airsim uses wxyz
        gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg->camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

// todo support multiple gimbal commands
// 1. find quaternion of default gimbal pose
// 2. forward multiply with quaternion equivalent to desired euler commands (in degrees)
// 3. call airsim client's setCameraPose which sets camera pose wrt world (or takeoff?) ned frame. todo
void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr gimbal_angle_euler_cmd_msg)
{
    try {
        tf2::Quaternion quat_control_cmd;
        quat_control_cmd.setRPY(math_common::deg2rad(gimbal_angle_euler_cmd_msg->roll), math_common::deg2rad(gimbal_angle_euler_cmd_msg->pitch), math_common::deg2rad(gimbal_angle_euler_cmd_msg->yaw));
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg->camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

airsim_interfaces::msg::CarState AirsimROSWrapper::get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    airsim_interfaces::msg::CarState state_msg;
    const auto odo = get_odom_msg_from_car_state(car_state);

    state_msg.pose = odo.pose;
    state_msg.twist = odo.twist;
    state_msg.speed = car_state.speed;
    state_msg.gear = car_state.gear;
    state_msg.rpm = car_state.rpm;
    state_msg.maxrpm = car_state.maxrpm;
    state_msg.handbrake = car_state.handbrake;
    state_msg.header.stamp = rclcpp::Time(car_state.timestamp);

    return state_msg;
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_kinematic_state(const msr::airlib::Kinematics::State& kinematics_estimated) const
{
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x = kinematics_estimated.pose.position.x();
    odom_msg.pose.pose.position.y = kinematics_estimated.pose.position.y();
    odom_msg.pose.pose.position.z = kinematics_estimated.pose.position.z();
    odom_msg.pose.pose.orientation.x = kinematics_estimated.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = kinematics_estimated.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = kinematics_estimated.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = kinematics_estimated.pose.orientation.w();

    odom_msg.twist.twist.linear.x = kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x = kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y = kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z = kinematics_estimated.twist.angular.z();

    if (isENU_) {
        std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
        odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
        std::swap(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y);
        odom_msg.pose.pose.orientation.z = -odom_msg.pose.pose.orientation.z;
        std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
        odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
        std::swap(odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y);
        odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    }

    return odom_msg;
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    return get_odom_msg_from_kinematic_state(car_state.kinematics_estimated);
}

nav_msgs::msg::Odometry AirsimROSWrapper::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& drone_state) const
{
    return get_odom_msg_from_kinematic_state(drone_state.kinematics_estimated);
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::msg::PointCloud2 AirsimROSWrapper::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = rclcpp::Time(lidar_data.time_stamp);
    lidar_msg.header.frame_id = vehicle_name + "/" + sensor_name;

    if (lidar_data.point_cloud.size() > 3) {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x";
        lidar_msg.fields[1].name = "y";
        lidar_msg.fields[2].name = "z";

        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            lidar_msg.fields[d].count = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
        std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);

        if (isENU_) {
            try {
                sensor_msgs::msg::PointCloud2 lidar_msg_enu;
                auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1));
                tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);

                lidar_msg_enu.header.stamp = lidar_msg.header.stamp;
                lidar_msg_enu.header.frame_id = lidar_msg.header.frame_id;

                lidar_msg = std::move(lidar_msg_enu);
            }
            catch (tf2::TransformException& ex) {
                RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
                rclcpp::Rate(1.0).sleep();
            }
        }
    }
    else {
        // msg = []
    }

    return lidar_msg;
}

airsim_interfaces::msg::Environment AirsimROSWrapper::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    airsim_interfaces::msg::Environment env_msg;
    env_msg.position.x = env_data.position.x();
    env_msg.position.y = env_data.position.y();
    env_msg.position.z = env_data.position.z();
    env_msg.geo_point.latitude = env_data.geo_point.latitude;
    env_msg.geo_point.longitude = env_data.geo_point.longitude;
    env_msg.geo_point.altitude = env_data.geo_point.altitude;
    env_msg.gravity.x = env_data.gravity.x();
    env_msg.gravity.y = env_data.gravity.y();
    env_msg.gravity.z = env_data.gravity.z();
    env_msg.air_pressure = env_data.air_pressure;
    env_msg.temperature = env_data.temperature;
    env_msg.air_density = env_data.temperature;

    return env_msg;
}

sensor_msgs::msg::MagneticField AirsimROSWrapper::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    mag_msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    mag_msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    std::copy(std::begin(mag_data.magnetic_field_covariance),
              std::end(mag_data.magnetic_field_covariance),
              std::begin(mag_msg.magnetic_field_covariance));
    mag_msg.header.stamp = rclcpp::Time(mag_data.time_stamp);

    return mag_msg;
}

// todo covariances
sensor_msgs::msg::NavSatFix AirsimROSWrapper::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = rclcpp::Time(gps_data.time_stamp);
    gps_msg.latitude = gps_data.gnss.geo_point.latitude;
    gps_msg.longitude = gps_data.gnss.geo_point.longitude;
    gps_msg.altitude = gps_data.gnss.geo_point.altitude;
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    gps_msg.status.status = gps_data.gnss.fix_type;
    // gps_msg.position_covariance_type =
    // gps_msg.position_covariance =

    return gps_msg;
}

sensor_msgs::msg::Range AirsimROSWrapper::get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::msg::Range dist_msg;
    dist_msg.header.stamp = rclcpp::Time(dist_data.time_stamp);
    dist_msg.range = dist_data.distance;
    dist_msg.min_range = dist_data.min_distance;
    dist_msg.max_range = dist_data.max_distance;

    return dist_msg;
}

airsim_interfaces::msg::Altimeter AirsimROSWrapper::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    airsim_interfaces::msg::Altimeter alt_msg;
    alt_msg.header.stamp = rclcpp::Time(alt_data.time_stamp);
    alt_msg.altitude = alt_data.altitude;
    alt_msg.pressure = alt_data.pressure;
    alt_msg.qnh = alt_data.qnh;

    return alt_msg;
}

// todo covariances
sensor_msgs::msg::Imu AirsimROSWrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple drones
    imu_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // meters/s2^m
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}

void AirsimROSWrapper::publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.child_frame_id = odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
}

// Publish map -> odom transform (identity by default, can be modified for localization drift)
void AirsimROSWrapper::publish_map_to_odom_tf(const rclcpp::Time& stamp, const std::string& vehicle_name)
{
    unused(vehicle_name); // Not used since we don't prefix frame names
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = stamp;
    map_to_odom_tf.header.frame_id = world_frame_id_;
    map_to_odom_tf.child_frame_id = odom_frame_id_; // No vehicle prefix
    // Identity transform by default (no drift between map and odom)
    map_to_odom_tf.transform.translation.x = 0.0;
    map_to_odom_tf.transform.translation.y = 0.0;
    map_to_odom_tf.transform.translation.z = 0.0;
    map_to_odom_tf.transform.rotation.x = 0.0;
    map_to_odom_tf.transform.rotation.y = 0.0;
    map_to_odom_tf.transform.rotation.z = 0.0;
    map_to_odom_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_odom_tf);
}

// Publish odom -> base_link transform (drone pose)
void AirsimROSWrapper::publish_odom_to_base_link_tf(const nav_msgs::msg::Odometry& odom_msg, const std::string& vehicle_name)
{
    unused(vehicle_name); // Not used since we don't prefix frame names
    geometry_msgs::msg::TransformStamped odom_to_base_link_tf;
    odom_to_base_link_tf.header.stamp = odom_msg.header.stamp;
    odom_to_base_link_tf.header.frame_id = odom_frame_id_; // No vehicle prefix
    odom_to_base_link_tf.child_frame_id = base_link_frame_id_; // No vehicle prefix
    odom_to_base_link_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_to_base_link_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_to_base_link_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_to_base_link_tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_to_base_link_tf);
}

airsim_interfaces::msg::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    airsim_interfaces::msg::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

sensor_msgs::msg::NavSatFix AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

msr::airlib::GeoPoint AirsimROSWrapper::get_origin_geo_point() const
{
    msr::airlib::HomeGeoPoint geo_point = AirSimSettings::singleton().origin_geopoint;
    return geo_point.home_geo_point;
}

VelCmd AirsimROSWrapper::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg) const
{
    VelCmd vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    return vel_cmd;
}

VelCmd AirsimROSWrapper::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, const msr::airlib::Quaternionr& airlib_quat) const
{
    VelCmd vel_cmd;
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(airlib_quat)).getRPY(roll, pitch, yaw); // ros uses xyzw

    // todo do actual body frame?
    vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw)); //body frame
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate = true;
    // airsim uses degrees
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);

    return vel_cmd;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll, rotation.pitch, rotation.yaw);
    transform.rotation.x = quat.x();
    transform.rotation.y = quat.y();
    transform.rotation.z = quat.z();
    transform.rotation.w = quat.w();

    return transform;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();

    return transform;
}

void AirsimROSWrapper::drone_state_timer_cb()
{
    try {
        // todo this is global origin
        origin_geo_point_pub_->publish(origin_geo_point_msg_);

        // get the basic vehicle pose and environmental state
        const auto now = update_state();

        // on init, will publish 0 to /clock as expected for use_sim_time compatibility
        if (!airsim_client_->simIsPaused()) {
            // airsim_client needs to provide the simulation time in a future version of the API
            ros_clock_.clock = now;
        }
        // publish the simulation clock
        if (publish_clock_) {
            clock_pub_->publish(ros_clock_);
        }

        // publish vehicle state, odom, and all basic sensor types
        publish_vehicle_state();

        // send any commands out to the vehicles
        update_commands();
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
    }
}

void AirsimROSWrapper::update_and_publish_static_transforms(VehicleROS* vehicle_ros)
{
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec_.empty()) {
        for (auto& static_tf_msg : vehicle_ros->static_tf_msg_vec_) {
            static_tf_msg.header.stamp = vehicle_ros->stamp_;
            static_tf_pub_->sendTransform(static_tf_msg);
        }
    }
}

rclcpp::Time AirsimROSWrapper::update_state()
{
    bool got_sim_time = false;
    rclcpp::Time curr_ros_time = nh_->now();

    //should be easier way to get the sim time through API, something like:
    //msr::airlib::Environment::State env = airsim_client_->simGetGroundTruthEnvironment("");
    //curr_ros_time = rclcpp::Time(env.clock().nowNanos());

    // iterate over drones
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        rclcpp::Time vehicle_time;
        // get drone state from airsim
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // vehicle environment, we can get ambient temperature here and other truths
        auto env_data = airsim_client_->simGetGroundTruthEnvironment(vehicle_ros->vehicle_name_);

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get());
            drone->curr_drone_state_ = rpc->getMultirotorState(vehicle_ros->vehicle_name_);

            vehicle_time = rclcpp::Time(drone->curr_drone_state_.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(drone->curr_drone_state_.gps_location);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            vehicle_ros->curr_odom_ = get_odom_msg_from_multirotor_state(drone->curr_drone_state_);
        }
        else {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            auto rpc = static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get());
            car->curr_car_state_ = rpc->getCarState(vehicle_ros->vehicle_name_);

            vehicle_time = rclcpp::Time(car->curr_car_state_.timestamp);
            if (!got_sim_time) {
                curr_ros_time = vehicle_time;
                got_sim_time = true;
            }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            vehicle_ros->curr_odom_ = get_odom_msg_from_car_state(car->curr_car_state_);

            airsim_interfaces::msg::CarState state_msg = get_roscarstate_msg_from_car_state(car->curr_car_state_);
            state_msg.header.frame_id = vehicle_ros->vehicle_name_;
            car->car_state_msg_ = state_msg;
        }

        vehicle_ros->stamp_ = vehicle_time;

        airsim_interfaces::msg::Environment env_msg = get_environment_msg_from_airsim(env_data);
        env_msg.header.frame_id = vehicle_ros->vehicle_name_;
        env_msg.header.stamp = vehicle_time;
        vehicle_ros->env_msg_ = env_msg;

        // convert airsim drone state to ROS msgs
        vehicle_ros->curr_odom_.header.frame_id = odom_frame_id_; // Parent frame: odom (no vehicle prefix)
        vehicle_ros->curr_odom_.child_frame_id = base_link_frame_id_; // Child frame: base_link (no vehicle prefix)
        vehicle_ros->curr_odom_.header.stamp = vehicle_time;
    }

    return curr_ros_time;
}

void AirsimROSWrapper::publish_vehicle_state()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        // simulation environment truth
        vehicle_ros->env_pub_->publish(vehicle_ros->env_msg_);

        if (airsim_mode_ == AIRSIM_MODE::CAR) {
            // dashboard reading from car, RPM, gear, etc
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->car_state_pub_->publish(car->car_state_msg_);
        }

        // odom and transforms
        // New TF tree: map -> odom -> base_link -> camera_link -> camera_*_frames
        vehicle_ros->odom_local_pub_->publish(vehicle_ros->curr_odom_);
        publish_map_to_odom_tf(vehicle_ros->stamp_, vehicle_ros->vehicle_name_);
        publish_odom_to_base_link_tf(vehicle_ros->curr_odom_, vehicle_ros->vehicle_name_);

        // ground truth GPS position from sim/HITL
        vehicle_ros->global_gps_pub_->publish(vehicle_ros->gps_sensor_msg_);

        for (auto& sensor_publisher : vehicle_ros->barometer_pubs_) {
            auto baro_data = airsim_client_->getBarometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            airsim_interfaces::msg::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
            alt_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(alt_msg);
        }

        // IMU is now published at 200Hz via dedicated imu_timer_cb() with separate RPC client

        for (auto& sensor_publisher : vehicle_ros->distance_pubs_) {
            auto distance_data = airsim_client_->getDistanceSensorData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::Range dist_msg = get_range_from_airsim(distance_data);
            dist_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(dist_msg);
        }
        for (auto& sensor_publisher : vehicle_ros->gps_pubs_) {
            auto gps_data = airsim_client_->getGpsData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::NavSatFix gps_msg = get_gps_msg_from_airsim(gps_data);
            gps_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(gps_msg);
        }
        for (auto& sensor_publisher : vehicle_ros->magnetometer_pubs_) {
            auto mag_data = airsim_client_->getMagnetometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::MagneticField mag_msg = get_mag_msg_from_airsim(mag_data);
            mag_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(mag_msg);
        }

        update_and_publish_static_transforms(vehicle_ros.get());
    }
}

void AirsimROSWrapper::update_commands()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;

        if (airsim_mode_ == AIRSIM_MODE::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());

            // send control commands from the last callback to airsim
            if (drone->has_vel_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                static_cast<msr::airlib::MultirotorRpcLibClient*>(airsim_client_.get())->moveByVelocityAsync(drone->vel_cmd_.x, drone->vel_cmd_.y, drone->vel_cmd_.z, vel_cmd_duration_, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, drone->vel_cmd_.yaw_mode, drone->vehicle_name_);
            }
            drone->has_vel_cmd_ = false;
        }
        else {
            // send control commands from the last callback to airsim
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            if (car->has_car_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                static_cast<msr::airlib::CarRpcLibClient*>(airsim_client_.get())->setCarControls(car->car_cmd_, vehicle_ros->vehicle_name_);
            }
            car->has_car_cmd_ = false;
        }
    }

    // Only camera rotation, no translation movement of camera
    if (has_gimbal_cmd_) {
        std::lock_guard<std::mutex> guard(control_mutex_);
        airsim_client_->simSetCameraPose(gimbal_cmd_.camera_name, get_airlib_pose(0, 0, 0, gimbal_cmd_.target_quat), gimbal_cmd_.vehicle_name);
    }

    has_gimbal_cmd_ = false;
}

// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS
void AirsimROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}

// if any nan's in camera pose, set them to match vehicle pose (which has already converted any potential nans to zeros)
void AirsimROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const
{
    if (std::isnan(camera_setting.position.x()))
        camera_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(camera_setting.position.y()))
        camera_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(camera_setting.position.z()))
        camera_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(camera_setting.rotation.yaw))
        camera_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(camera_setting.rotation.pitch))
        camera_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(camera_setting.rotation.roll))
        camera_setting.rotation.roll = vehicle_setting.rotation.roll;
}

void AirsimROSWrapper::convert_tf_msg_to_enu(geometry_msgs::msg::TransformStamped& tf_msg)
{
    std::swap(tf_msg.transform.translation.x, tf_msg.transform.translation.y);
    std::swap(tf_msg.transform.rotation.x, tf_msg.transform.rotation.y);
    tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
    tf_msg.transform.rotation.z = -tf_msg.transform.rotation.z;
}

geometry_msgs::msg::Transform AirsimROSWrapper::get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const
{
    geometry_msgs::msg::Transform optical_tf = body_tf; //same translation
    auto opticalQ = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
    if (isENU_)
        opticalQ *= msr::airlib::Quaternionr(0.7071068, -0.7071068, 0, 0); //CamOptical in CamBodyENU is rmat[1,0,0;0,0,-1;0,1,0]==xyzw[-0.7071068,0,0,0.7071068]
    else
        opticalQ *= msr::airlib::Quaternionr(0.5, 0.5, 0.5, 0.5); //CamOptical in CamBodyNED is rmat[0,0,1;1,0,0;0,1,0]==xyzw[0.5,0.5,0.5,0.5]
    optical_tf.rotation.w = opticalQ.w();
    optical_tf.rotation.x = opticalQ.x();
    optical_tf.rotation.y = opticalQ.y();
    optical_tf.rotation.z = opticalQ.z();
    return optical_tf;
}

void AirsimROSWrapper::append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::msg::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = nh_->now();
    vehicle_tf_msg.child_frame_id = vehicle_ros->vehicle_name_;
    vehicle_tf_msg.transform = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);

    if (isENU_) {
        convert_tf_msg_to_enu(vehicle_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(vehicle_tf_msg);
}

void AirsimROSWrapper::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting)
{
    geometry_msgs::msg::TransformStamped lidar_tf_msg;
    // Lidar is attached to base_link (drone body frame)
    lidar_tf_msg.header.frame_id = base_link_frame_id_; // No vehicle prefix
    lidar_tf_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + lidar_name;
    lidar_tf_msg.transform = get_transform_msg_from_airsim(lidar_setting.relative_pose.position, lidar_setting.relative_pose.orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(lidar_tf_msg);
    }

    vehicle_ros->static_tf_msg_vec_.emplace_back(lidar_tf_msg);
}

void AirsimROSWrapper::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped static_cam_tf_body_msg;
    // Static camera TF is now relative to base_link
    static_cam_tf_body_msg.header.frame_id = base_link_frame_id_; // No vehicle prefix
    static_cam_tf_body_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_body/static";
    static_cam_tf_body_msg.transform = get_transform_msg_from_airsim(camera_setting.position, camera_setting.rotation);

    if (isENU_) {
        convert_tf_msg_to_enu(static_cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_optical/static";
    static_cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(static_cam_tf_body_msg.transform);

    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_optical_msg);
}

// Append static transform from base_link to camera_link
// This represents the physical mounting position of the camera on the drone
// camera_link is published without vehicle prefix to match RealSense convention
void AirsimROSWrapper::append_static_base_link_to_camera_link_tf(VehicleROS* vehicle_ros)
{
    geometry_msgs::msg::TransformStamped base_to_camera_link_tf;
    base_to_camera_link_tf.header.frame_id = base_link_frame_id_; // No vehicle prefix
    // camera_link without vehicle prefix to match RealSense convention
    base_to_camera_link_tf.child_frame_id = camera_link_frame_id_;
    // Default identity transform - modify these values to match your camera mount position
    // relative to the drone's center of gravity (base_link)
    base_to_camera_link_tf.transform.translation.x = 0.0; // Forward offset from CoG
    base_to_camera_link_tf.transform.translation.y = 0.0; // Left offset from CoG
    base_to_camera_link_tf.transform.translation.z = 0.0; // Down offset from CoG (NED) or Up (ENU)
    base_to_camera_link_tf.transform.rotation.x = 0.0;
    base_to_camera_link_tf.transform.rotation.y = 0.0;
    base_to_camera_link_tf.transform.rotation.z = 0.0;
    base_to_camera_link_tf.transform.rotation.w = 1.0;

    vehicle_ros->static_tf_msg_vec_.emplace_back(base_to_camera_link_tf);
}

// Append static transforms for camera sub-frames (infra1, infra2, depth, color, etc.)
// These are relative to camera_link, mimicking RealSense camera frame structure exactly
// All camera frames use clean names without vehicle prefix to match RealSense convention
void AirsimROSWrapper::append_static_camera_subframes_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    // camera_link without vehicle prefix to match RealSense convention
    const std::string camera_link_id = camera_link_frame_id_;

    // Get the camera's transform from settings (identity for RealSense-style setup)
    geometry_msgs::msg::Transform cam_transform = get_transform_msg_from_airsim(camera_setting.position, camera_setting.rotation);
    if (isENU_) {
        geometry_msgs::msg::TransformStamped temp_tf;
        temp_tf.transform = cam_transform;
        convert_tf_msg_to_enu(temp_tf);
        cam_transform = temp_tf.transform;
    }

    // Create identity transform for optical frame rotation (Z forward, X right, Y down)
    geometry_msgs::msg::Transform optical_rotation;
    optical_rotation.translation.x = 0.0;
    optical_rotation.translation.y = 0.0;
    optical_rotation.translation.z = 0.0;
    if (isENU_) {
        // ENU: rotate to get Z forward, X right, Y down
        optical_rotation.rotation.x = -0.5;
        optical_rotation.rotation.y = 0.5;
        optical_rotation.rotation.z = -0.5;
        optical_rotation.rotation.w = 0.5;
    }
    else {
        // NED: rotate to get optical frame
        optical_rotation.rotation.x = 0.5;
        optical_rotation.rotation.y = 0.5;
        optical_rotation.rotation.z = 0.5;
        optical_rotation.rotation.w = 0.5;
    }

    // camera_depth_frame: parent = camera_link
    geometry_msgs::msg::TransformStamped camera_depth_frame_tf;
    camera_depth_frame_tf.header.frame_id = camera_link_id;
    camera_depth_frame_tf.child_frame_id = "camera_depth_frame";
    camera_depth_frame_tf.transform = cam_transform;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_depth_frame_tf);

    // camera_depth_optical_frame: parent = camera_depth_frame
    geometry_msgs::msg::TransformStamped camera_depth_optical_tf;
    camera_depth_optical_tf.header.frame_id = "camera_depth_frame";
    camera_depth_optical_tf.child_frame_id = "camera_depth_optical_frame";
    camera_depth_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_depth_optical_tf);

    // camera_infra1_frame: parent = camera_link
    geometry_msgs::msg::TransformStamped camera_infra1_frame_tf;
    camera_infra1_frame_tf.header.frame_id = camera_link_id;
    camera_infra1_frame_tf.child_frame_id = "camera_infra1_frame";
    camera_infra1_frame_tf.transform = cam_transform;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_infra1_frame_tf);

    // camera_aligned_depth_to_infra1_frame: parent = camera_link
    // This frame aligns depth to infra1 for stereo processing
    geometry_msgs::msg::TransformStamped camera_aligned_depth_to_infra1_frame_tf;
    camera_aligned_depth_to_infra1_frame_tf.header.frame_id = camera_link_id;
    camera_aligned_depth_to_infra1_frame_tf.child_frame_id = "camera_aligned_depth_to_infra1_frame";
    camera_aligned_depth_to_infra1_frame_tf.transform = cam_transform; // Same as depth frame
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_aligned_depth_to_infra1_frame_tf);

    // camera_infra1_optical_frame: parent = camera_aligned_depth_to_infra1_frame (RealSense convention)
    geometry_msgs::msg::TransformStamped camera_infra1_optical_tf;
    camera_infra1_optical_tf.header.frame_id = "camera_aligned_depth_to_infra1_frame";
    camera_infra1_optical_tf.child_frame_id = "camera_infra1_optical_frame";
    camera_infra1_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_infra1_optical_tf);

    // camera_infra2_frame: parent = camera_link (offset by stereo baseline, typically ~50mm)
    geometry_msgs::msg::TransformStamped camera_infra2_frame_tf;
    camera_infra2_frame_tf.header.frame_id = camera_link_id;
    camera_infra2_frame_tf.child_frame_id = "camera_infra2_frame";
    camera_infra2_frame_tf.transform = cam_transform;
    // Add stereo baseline offset (adjust this value based on your simulated camera)
    // RealSense D435 has ~95mm baseline, but for simulation 50mm is typical
    camera_infra2_frame_tf.transform.translation.y += 0.05; // 50mm baseline (positive Y is left in ROS)
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_infra2_frame_tf);

    // camera_infra2_optical_frame: parent = camera_infra2_frame
    geometry_msgs::msg::TransformStamped camera_infra2_optical_tf;
    camera_infra2_optical_tf.header.frame_id = "camera_infra2_frame";
    camera_infra2_optical_tf.child_frame_id = "camera_infra2_optical_frame";
    camera_infra2_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_infra2_optical_tf);

    // camera_color_frame: parent = camera_link
    geometry_msgs::msg::TransformStamped camera_color_frame_tf;
    camera_color_frame_tf.header.frame_id = camera_link_id;
    camera_color_frame_tf.child_frame_id = "camera_color_frame";
    camera_color_frame_tf.transform = cam_transform;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_color_frame_tf);

    // camera_color_optical_frame: parent = camera_color_frame
    geometry_msgs::msg::TransformStamped camera_color_optical_tf;
    camera_color_optical_tf.header.frame_id = "camera_color_frame";
    camera_color_optical_tf.child_frame_id = "camera_color_optical_frame";
    camera_color_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_color_optical_tf);

    // IMU-related frames (RealSense has built-in IMU)
    // camera_accel_frame: parent = camera_link
    geometry_msgs::msg::TransformStamped camera_accel_frame_tf;
    camera_accel_frame_tf.header.frame_id = camera_link_id;
    camera_accel_frame_tf.child_frame_id = "camera_accel_frame";
    camera_accel_frame_tf.transform = cam_transform; // Same position as camera
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_accel_frame_tf);

    // camera_accel_optical_frame: parent = camera_accel_frame
    geometry_msgs::msg::TransformStamped camera_accel_optical_tf;
    camera_accel_optical_tf.header.frame_id = "camera_accel_frame";
    camera_accel_optical_tf.child_frame_id = "camera_accel_optical_frame";
    camera_accel_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_accel_optical_tf);

    // camera_gyro_frame: parent = camera_link
    geometry_msgs::msg::TransformStamped camera_gyro_frame_tf;
    camera_gyro_frame_tf.header.frame_id = camera_link_id;
    camera_gyro_frame_tf.child_frame_id = "camera_gyro_frame";
    camera_gyro_frame_tf.transform = cam_transform; // Same position as camera
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_gyro_frame_tf);

    // camera_gyro_optical_frame: parent = camera_gyro_frame
    geometry_msgs::msg::TransformStamped camera_gyro_optical_tf;
    camera_gyro_optical_tf.header.frame_id = "camera_gyro_frame";
    camera_gyro_optical_tf.child_frame_id = "camera_gyro_optical_frame";
    camera_gyro_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_gyro_optical_tf);

    // camera_imu_frame: parent = camera_gyro_frame (RealSense convention)
    geometry_msgs::msg::TransformStamped camera_imu_frame_tf;
    camera_imu_frame_tf.header.frame_id = "camera_gyro_frame";
    camera_imu_frame_tf.child_frame_id = "camera_imu_frame";
    camera_imu_frame_tf.transform.translation.x = 0.0;
    camera_imu_frame_tf.transform.translation.y = 0.0;
    camera_imu_frame_tf.transform.translation.z = 0.0;
    camera_imu_frame_tf.transform.rotation.x = 0.0;
    camera_imu_frame_tf.transform.rotation.y = 0.0;
    camera_imu_frame_tf.transform.rotation.z = 0.0;
    camera_imu_frame_tf.transform.rotation.w = 1.0; // Identity (IMU is at gyro location)
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_imu_frame_tf);

    // camera_imu_optical_frame: parent = camera_imu_frame
    geometry_msgs::msg::TransformStamped camera_imu_optical_tf;
    camera_imu_optical_tf.header.frame_id = "camera_imu_frame";
    camera_imu_optical_tf.child_frame_id = "camera_imu_optical_frame";
    camera_imu_optical_tf.transform = optical_rotation;
    vehicle_ros->static_tf_msg_vec_.emplace_back(camera_imu_optical_tf);
}

void AirsimROSWrapper::img_response_timer_cb()
{
    try {
        size_t num_cameras = airsim_img_request_vehicle_name_pair_vec_.size();

        if (num_cameras == 0) return;

        // Launch parallel image fetch requests - each camera uses its own RPC client
        std::vector<std::future<std::vector<ImageResponse>>> futures;
        futures.reserve(num_cameras);

        for (size_t i = 0; i < num_cameras; ++i) {
            const auto& request_pair = airsim_img_request_vehicle_name_pair_vec_[i];
            auto* client = airsim_client_images_vec_[i].get();

            futures.push_back(std::async(std::launch::async,
                                         [client, &request_pair]() {
                                             return client->simGetImages(request_pair.first, request_pair.second);
                                         }));
        }

        // Collect results and publish - maintains original order
        int image_response_idx = 0;
        for (size_t i = 0; i < num_cameras; ++i) {
            const auto& request_pair = airsim_img_request_vehicle_name_pair_vec_[i];
            std::vector<ImageResponse> img_response = futures[i].get();

            if (img_response.size() == request_pair.first.size()) {
                process_and_publish_img_response(img_response, image_response_idx, request_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get image response.\n%s", msg.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Exception in image callback: %s", e.what());
    }
}

void AirsimROSWrapper::lidar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            if (!vehicle_name_ptr_pair.second->lidar_pubs_.empty()) {
                for (auto& lidar_publisher : vehicle_name_ptr_pair.second->lidar_pubs_) {
                    auto lidar_data = airsim_client_lidar_.getLidarData(lidar_publisher.sensor_name, vehicle_name_ptr_pair.first);
                    sensor_msgs::msg::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data, vehicle_name_ptr_pair.first, lidar_publisher.sensor_name);
                    lidar_publisher.publisher->publish(lidar_msg);
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get lidar response.\n%s", msg.c_str());
    }
}

// High-frequency IMU callback at 200Hz with dedicated RPC client for parallel execution
void AirsimROSWrapper::imu_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            auto& vehicle_ros = vehicle_name_ptr_pair.second;

            // Process each IMU sensor
            for (size_t i = 0; i < vehicle_ros->imu_pubs_.size(); ++i) {
                auto& sensor_publisher = vehicle_ros->imu_pubs_[i];
                auto imu_data = airsim_client_imu_.getImuData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);

                // Get full IMU message for legacy publisher
                sensor_msgs::msg::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
                imu_msg.header.frame_id = vehicle_ros->vehicle_name_;
                sensor_publisher.publisher->publish(imu_msg);

                // Publish separate gyro message (RealSense-style)
                if (i < vehicle_ros->gyro_pubs_.size()) {
                    sensor_msgs::msg::Imu gyro_msg;
                    ImuMessage_AddDefaultValues(gyro_msg, "camera_gyro_optical_frame");
                    gyro_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);
                    gyro_msg.angular_velocity.x = imu_data.angular_velocity.x();
                    gyro_msg.angular_velocity.y = imu_data.angular_velocity.y();
                    gyro_msg.angular_velocity.z = imu_data.angular_velocity.z();
                    // Linear acceleration fields remain zero for gyro-only message
                    vehicle_ros->gyro_pubs_[i].publisher->publish(gyro_msg);

                    // Store gyro data for unified IMU
                    if (unite_imu_method_ > 0) {
                        std::lock_guard<std::mutex> lock(imu_sync_mutex_);
                        CimuData gyro_data(0, imu_data.angular_velocity, imu_data.time_stamp);
                        std::deque<sensor_msgs::msg::Imu> imu_msgs;
                        if (unite_imu_method_ == 1) {
                            FillImuData_Copy(gyro_data, imu_msgs);
                        }
                        else if (unite_imu_method_ == 2) {
                            // For linear interpolation, we need to process accel data first
                            // So we'll process it when accel arrives
                        }
                        while (!imu_msgs.empty()) {
                            sensor_msgs::msg::Imu unified_msg = imu_msgs.front();
                            imu_msgs.pop_front();
                            if (unified_imu_pub_) {
                                unified_imu_pub_->publish(unified_msg);
                            }
                        }
                    }
                }

                // Publish separate accel message (RealSense-style)
                if (i < vehicle_ros->accel_pubs_.size()) {
                    sensor_msgs::msg::Imu accel_msg;
                    ImuMessage_AddDefaultValues(accel_msg, "camera_accel_optical_frame");
                    accel_msg.header.stamp = rclcpp::Time(imu_data.time_stamp);
                    accel_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
                    accel_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
                    accel_msg.linear_acceleration.z = imu_data.linear_acceleration.z();
                    // Angular velocity fields remain zero for accel-only message
                    vehicle_ros->accel_pubs_[i].publisher->publish(accel_msg);

                    // Store accel data for unified IMU
                    if (unite_imu_method_ > 0) {
                        std::lock_guard<std::mutex> lock(imu_sync_mutex_);
                        CimuData accel_data(1, imu_data.linear_acceleration, imu_data.time_stamp);
                        last_accel_data_ = accel_data;
                        if (unite_imu_method_ == 2) {
                            // For linear interpolation, process accel data to match with gyro timestamps
                            std::deque<sensor_msgs::msg::Imu> imu_msgs;
                            FillImuData_LinearInterpolation(accel_data, imu_msgs);
                            while (!imu_msgs.empty()) {
                                sensor_msgs::msg::Imu unified_msg = imu_msgs.front();
                                imu_msgs.pop_front();
                                if (unified_imu_pub_) {
                                    unified_imu_pub_->publish(unified_msg);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get IMU response.\n%s", msg.c_str());
    }
}

// IMU synchronization methods
sensor_msgs::msg::Imu AirsimROSWrapper::CreateUnitedImuMessage(const CimuData& accel_data, const CimuData& gyro_data) const
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Time(gyro_data.m_time_ns);
    imu_msg.header.frame_id = "camera_imu_optical_frame";

    imu_msg.angular_velocity.x = gyro_data.m_data.x();
    imu_msg.angular_velocity.y = gyro_data.m_data.y();
    imu_msg.angular_velocity.z = gyro_data.m_data.z();

    imu_msg.linear_acceleration.x = accel_data.m_data.x();
    imu_msg.linear_acceleration.y = accel_data.m_data.y();
    imu_msg.linear_acceleration.z = accel_data.m_data.z();

    ImuMessage_AddDefaultValues(imu_msg, "camera_imu_optical_frame");
    return imu_msg;
}

void AirsimROSWrapper::FillImuData_Copy(const CimuData& imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    // GYRO type = 0, ACCEL type = 1
    if (imu_data.m_type == 1) { // ACCEL
        last_accel_data_ = imu_data;
        return;
    }
    if (!last_accel_data_.is_set()) {
        return;
    }

    imu_msgs.push_back(CreateUnitedImuMessage(last_accel_data_, imu_data));
}

void AirsimROSWrapper::FillImuData_LinearInterpolation(const CimuData& imu_data, std::deque<sensor_msgs::msg::Imu>& imu_msgs)
{
    imu_history_.push_back(imu_data);
    imu_msgs.clear();

    // Need at least 3 samples for interpolation
    if (imu_history_.size() < 3) {
        return;
    }

    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;
    CimuData last_imu;

    while (!imu_history_.empty()) {
        crnt_imu = imu_history_.front();
        imu_history_.pop_front();

        if (!accel0.is_set() && crnt_imu.m_type == 1) { // ACCEL
            accel0 = crnt_imu;
        }
        else if (accel0.is_set() && crnt_imu.m_type == 1) { // ACCEL
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ns - accel0.m_time_ns;
            if (dt > 0) {
                while (!gyros_data.empty()) {
                    CimuData crnt_gyro = gyros_data.front();
                    gyros_data.pop_front();
                    const double alpha = (crnt_gyro.m_time_ns - accel0.m_time_ns) / dt;

                    // Linear interpolation: lerp(a, b, t) = a * (1-t) + b * t
                    msr::airlib::Vector3r interpolated_accel = accel0.m_data * (1.0 - alpha) + accel1.m_data * alpha;
                    CimuData crnt_accel(1, interpolated_accel, crnt_gyro.m_time_ns);
                    imu_msgs.push_back(CreateUnitedImuMessage(crnt_accel, crnt_gyro));
                }
            }
            accel0 = accel1;
        }
        else if (accel0.is_set() && crnt_imu.m_time_ns >= accel0.m_time_ns && crnt_imu.m_type == 0) { // GYRO
            gyros_data.push_back(crnt_imu);
        }
        last_imu = crnt_imu;
    }
    if (last_imu.is_set()) {
        imu_history_.push_back(last_imu);
    }
}

void AirsimROSWrapper::ImuMessage_AddDefaultValues(sensor_msgs::msg::Imu& imu_msg, const std::string& frame_id) const
{
    imu_msg.header.frame_id = frame_id;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 0.0;

    // Default covariance values (can be made configurable)
    const double linear_accel_cov = 0.01;
    const double angular_velocity_cov = 0.01;

    imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    imu_msg.linear_acceleration_covariance = { linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov };
    imu_msg.angular_velocity_covariance = { angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov };
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                                     const rclcpp::Time curr_ros_time,
                                                                                     const std::string frame_id)
{
    unused(curr_ros_time);
    std::shared_ptr<sensor_msgs::msg::Image> img_msg_ptr = std::make_shared<sensor_msgs::msg::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.image_data_uint8.size() / img_response.height;
    img_msg_ptr->header.stamp = rclcpp::Time(img_response.time_stamp);
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "rgb8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                                           const rclcpp::Time curr_ros_time,
                                                                                           const std::string frame_id)
{
    unused(curr_ros_time);
    auto depth_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    depth_img_msg->width = img_response.width;
    depth_img_msg->height = img_response.height;

    // For DepthPlanar, convert to 16UC1 encoding (uint16 in millimeters)
    // This matches RealSense-style depth image format
    if (img_response.image_type == ImageType::DepthPlanar) {
        // Convert float depth (in meters) to uint16 (in millimeters)
        // Scale factor: 1000.0 to convert meters to millimeters
        // Clamp to uint16 range (0-65535 mm = 0-65.535 m)
        const float depth_scale = 1000.0f; // meters to millimeters
        const uint16_t max_depth_mm = 65535; // max value for uint16

        size_t num_pixels = img_response.image_data_float.size();
        depth_img_msg->data.resize(num_pixels * sizeof(uint16_t));
        uint16_t* depth_data = reinterpret_cast<uint16_t*>(depth_img_msg->data.data());

        for (size_t i = 0; i < num_pixels; ++i) {
            float depth_m = img_response.image_data_float[i];
            // Convert to millimeters and clamp to uint16 range
            float depth_mm = depth_m * depth_scale;
            if (depth_mm < 0.0f) {
                depth_data[i] = 0;
            }
            else if (depth_mm > max_depth_mm) {
                depth_data[i] = max_depth_mm;
            }
            else {
                depth_data[i] = static_cast<uint16_t>(depth_mm);
            }
        }

        depth_img_msg->encoding = "16UC1";
        depth_img_msg->step = img_response.width * sizeof(uint16_t); // 2 bytes per pixel
    }
    else {
        // For other depth types (DepthPerspective, DepthVis, DisparityNormalized), keep 32FC1
        depth_img_msg->data.resize(img_response.image_data_float.size() * sizeof(float));
        memcpy(depth_img_msg->data.data(), img_response.image_data_float.data(), depth_img_msg->data.size());
        depth_img_msg->encoding = "32FC1";
        depth_img_msg->step = depth_img_msg->data.size() / img_response.height;
    }

    depth_img_msg->is_bigendian = 0;
    depth_img_msg->header.stamp = rclcpp::Time(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::msg::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& stream_name,
                                                                 const CameraSetting& camera_setting,
                                                                 const CaptureSetting& capture_setting) const
{
    unused(camera_setting);
    sensor_msgs::msg::CameraInfo cam_info_msg;
    // Use RealSense frame naming convention: camera_{stream}_optical_frame
    cam_info_msg.header.frame_id = "camera_" + stream_name + "_optical_frame";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };

    // Set distortion model to plumb_bob with zero distortion coefficients (matching RealSense default)
    cam_info_msg.distortion_model = "plumb_bob";
    cam_info_msg.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    cam_info_msg.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    rclcpp::Time curr_ros_time = nh_->now();
    int img_response_idx_internal = img_response_idx;

    for (const auto& curr_img_response : img_response_vec) {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope.
        // Ideally, we should loop over cameras and then captures, and publish only one tf.
        publish_camera_tf(curr_img_response, curr_ros_time, vehicle_name, curr_img_response.camera_name);

        // todo simGetCameraInfo is wrong + also it's only for image type -1.
        // msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);

        // update timestamp of saved cam info msgs

        camera_info_msg_vec_[img_response_idx_internal].header.stamp = rclcpp::Time(curr_img_response.time_stamp);
        cam_info_pub_vec_[img_response_idx_internal]->publish(camera_info_msg_vec_[img_response_idx_internal]);

        // Get RealSense frame ID from camera_info (which was set with correct stream name)
        std::string frame_id = camera_info_msg_vec_[img_response_idx_internal].header.frame_id;

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            auto depth_img_msg = get_depth_img_msg_from_response(curr_img_response,
                                                                 curr_ros_time,
                                                                 frame_id);
            // Use raw publisher (no compression)
            image_pub_vec_[img_response_idx_internal]->publish(*depth_img_msg);
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            auto img_msg = get_img_msg_from_response(curr_img_response,
                                                     curr_ros_time,
                                                     frame_id);
            // Use raw publisher (no compression)
            image_pub_vec_[img_response_idx_internal]->publish(*img_msg);
        }
        img_response_idx_internal++;
    }
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame.
// We first do a change of basis to camera optical frame (Z forward, X right, Y down)
// Now publishes relative to base_link for proper TF tree: map -> odom -> base_link -> camera_link -> ...
void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    unused(ros_time);
    unused(frame_id); // Not used since we use base_link directly
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    // Camera is relative to base_link (through camera_link) - no vehicle prefix
    cam_tf_body_msg.header.frame_id = base_link_frame_id_;
    cam_tf_body_msg.child_frame_id = child_frame_id + "_body";
    cam_tf_body_msg.transform = get_transform_msg_from_airsim(img_response.camera_position, img_response.camera_orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_optical_msg.header.frame_id = base_link_frame_id_; // No vehicle prefix
    cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
    cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(cam_tf_body_msg.transform);

    tf_broadcaster_->sendTransform(cam_tf_body_msg);
    tf_broadcaster_->sendTransform(cam_tf_optical_msg);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m) const
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows * cols; ++i) {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::msg::CameraInfo& cam_info) const
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.k[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.r[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.p[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.d.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i) {
        cam_info.d[i] = D_data[i].as<float>();
    }
}
