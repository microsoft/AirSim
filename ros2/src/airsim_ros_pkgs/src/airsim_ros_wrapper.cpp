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
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

AirsimROSWrapper::AirsimROSWrapper(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_img, const std::shared_ptr<rclcpp::Node> nh_lidar, const std::string& host_ip)
    : is_used_lidar_timer_cb_queue_(false)
    , is_used_img_timer_cb_queue_(false)
    , airsim_settings_parser_(host_ip)
    , host_ip_(host_ip)
    , airsim_client_(nullptr)
    , airsim_client_images_(host_ip)
    , airsim_client_lidar_(host_ip)
    , nh_(nh)
    , nh_img_(nh_img)
    , nh_lidar_(nh_lidar)
    , isENU_(false)
    , publish_clock_(false)
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
        airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();

        for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            airsim_client_->enableApiControl(true, vehicle_name_ptr_pair.first); // todo expose as rosservice?
            airsim_client_->armDisarm(true, vehicle_name_ptr_pair.first); // todo exposes as rosservice?
        }

        origin_geo_point_ = get_origin_geo_point();
        // todo there's only one global origin geopoint for environment. but airsim API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a geopoint being assigned in the constructor. by?
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, something went wrong.\n%s", msg.c_str());
        rclcpp::shutdown();
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
    isENU_ = (odom_frame_id_ == ENU_ODOM_FRAME_ID);
    nh_->get_parameter_or("coordinate_system_enu", isENU_, isENU_);
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

    image_transport::ImageTransport image_transporter(nh_);

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

        vehicle_ros->odom_frame_id_ = curr_vehicle_name + "/" + odom_frame_id_;
        vehicle_ros->vehicle_name_ = curr_vehicle_name;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

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
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);
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

                    const std::string camera_topic = topic_prefix + "/" + curr_camera_name + "/" + image_type_int_to_string_map_.at(capture_setting.image_type);
                    image_pub_vec_.push_back(image_transporter.advertise(camera_topic, 1));
                    cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic + "/camera_info", 10));
                    camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
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
                    SensorPublisher<sensor_msgs::msg::Imu> sensor_publisher =
                        create_sensor_publisher<sensor_msgs::msg::Imu>("Imu", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/imu/" + sensor_name, 10);
                    vehicle_ros->imu_pubs_.emplace_back(sensor_publisher);
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

    // if >0 cameras, add one more thread for img_request_timer_cb
    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        nh_->get_parameter("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);

        airsim_img_response_timer_ = nh_img_->create_wall_timer(std::chrono::duration<double>(update_airsim_img_response_every_n_sec), std::bind(&AirsimROSWrapper::img_response_timer_cb, this));
        is_used_img_timer_cb_queue_ = true;
    }

    // lidars update on their own callback/thread at a given rate
    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec;
        nh_->get_parameter("update_lidar_every_n_sec", update_lidar_every_n_sec);
        airsim_lidar_update_timer_ = nh_lidar_->create_wall_timer(std::chrono::duration<double>(update_lidar_every_n_sec), std::bind(&AirsimROSWrapper::lidar_timer_cb, this));
        is_used_lidar_timer_cb_queue_ = true;
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
                auto transformStampedENU = tf_buffer_->lookupTransform(AIRSIM_FRAME_ID, vehicle_name, rclcpp::Time(0), std::chrono::nanoseconds(1));
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
        vehicle_ros->curr_odom_.header.frame_id = vehicle_ros->vehicle_name_;
        vehicle_ros->curr_odom_.child_frame_id = vehicle_ros->odom_frame_id_;
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
        vehicle_ros->odom_local_pub_->publish(vehicle_ros->curr_odom_);
        publish_odom_tf(vehicle_ros->curr_odom_);

        // ground truth GPS position from sim/HITL
        vehicle_ros->global_gps_pub_->publish(vehicle_ros->gps_sensor_msg_);

        for (auto& sensor_publisher : vehicle_ros->barometer_pubs_) {
            auto baro_data = airsim_client_->getBarometerData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            airsim_interfaces::msg::Altimeter alt_msg = get_altimeter_msg_from_airsim(baro_data);
            alt_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(alt_msg);
        }

        for (auto& sensor_publisher : vehicle_ros->imu_pubs_) {
            auto imu_data = airsim_client_->getImuData(sensor_publisher.sensor_name, vehicle_ros->vehicle_name_);
            sensor_msgs::msg::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
            imu_msg.header.frame_id = vehicle_ros->vehicle_name_;
            sensor_publisher.publisher->publish(imu_msg);
        }
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
    lidar_tf_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
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
    static_cam_tf_body_msg.header.frame_id = vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    static_cam_tf_body_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_body/static";
    static_cam_tf_body_msg.transform = get_transform_msg_from_airsim(camera_setting.position, camera_setting.rotation);

    if (isENU_) {
        convert_tf_msg_to_enu(static_cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped static_cam_tf_optical_msg = static_cam_tf_body_msg;
    static_cam_tf_optical_msg.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_optical/static";
    static_cam_tf_optical_msg.child_frame_id = camera_name + "_optical/static";
    static_cam_tf_optical_msg.transform = get_camera_optical_tf_from_body_tf(static_cam_tf_body_msg.transform);

    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_body_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(static_cam_tf_optical_msg);
}

void AirsimROSWrapper::img_response_timer_cb()
{
    try {
        int image_response_idx = 0;
        for (const auto& airsim_img_request_vehicle_name_pair : airsim_img_request_vehicle_name_pair_vec_) {
            const std::vector<ImageResponse>& img_response = airsim_client_images_.simGetImages(airsim_img_request_vehicle_name_pair.first, airsim_img_request_vehicle_name_pair.second);

            if (img_response.size() == airsim_img_request_vehicle_name_pair.first.size()) {
                process_and_publish_img_response(img_response, image_response_idx, airsim_img_request_vehicle_name_pair.second);
                image_response_idx += img_response.size();
            }
        }
    }

    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get image response.\n%s", msg.c_str());
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
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API, didn't get image response.\n%s", msg.c_str());
    }
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
    depth_img_msg->data.resize(img_response.image_data_float.size() * sizeof(float));
    memcpy(depth_img_msg->data.data(), img_response.image_data_float.data(), depth_img_msg->data.size());
    depth_img_msg->encoding = "32FC1";
    depth_img_msg->step = depth_img_msg->data.size() / img_response.height;
    depth_img_msg->is_bigendian = 0;
    depth_img_msg->header.stamp = rclcpp::Time(img_response.time_stamp);
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}

// todo have a special stereo pair mode and get projection matrix by calculating offset wrt drone body frame?
sensor_msgs::msg::CameraInfo AirsimROSWrapper::generate_cam_info(const std::string& camera_name,
                                                                 const CameraSetting& camera_setting,
                                                                 const CaptureSetting& capture_setting) const
{
    unused(camera_setting);
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = capture_setting.height;
    cam_info_msg.width = capture_setting.width;
    float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    cam_info_msg.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
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

        // DepthPlanar / DepthPerspective / DepthVis / DisparityNormalized
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[img_response_idx_internal].publish(get_depth_img_msg_from_response(curr_img_response,
                                                                                              curr_ros_time,
                                                                                              curr_img_response.camera_name + "_optical"));
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else {
            image_pub_vec_[img_response_idx_internal].publish(get_img_msg_from_response(curr_img_response,
                                                                                        curr_ros_time,
                                                                                        curr_img_response.camera_name + "_optical"));
        }
        img_response_idx_internal++;
    }
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame.
// We first do a change of basis to camera optical frame (Z forward, X right, Y down)
void AirsimROSWrapper::publish_camera_tf(const ImageResponse& img_response, const rclcpp::Time& ros_time, const std::string& frame_id, const std::string& child_frame_id)
{
    unused(ros_time);
    geometry_msgs::msg::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = frame_id + "/" + child_frame_id + "_body";
    cam_tf_body_msg.transform = get_transform_msg_from_airsim(img_response.camera_position, img_response.camera_orientation);

    if (isENU_) {
        convert_tf_msg_to_enu(cam_tf_body_msg);
    }

    geometry_msgs::msg::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = rclcpp::Time(img_response.time_stamp);
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = frame_id + "/" + child_frame_id + "_optical";
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
