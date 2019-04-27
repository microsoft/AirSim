#include <airsim_ros_wrapper.h>
#include <boost/make_shared.hpp>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(AirsimROSWrapper, nodelet::Nodelet)

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

void AirsimROSWrapper::onInit()
{
    initialize_airsim();
    initialize_ros();
    in_air_ = false;
    // intitialize placeholder control commands
    // vel_cmd_ = VelCmd();
    // gimbal_cmd_ = GimbalCmd();
}

void AirsimROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try
    {
        airsim_client_.confirmConnection();
        airsim_client_.enableApiControl(true); // todo expose as rosservice?
        airsim_client_.armDisarm(true); // todo expose as rosservice?
        home_geo_point_ = airsim_client_.getHomeGeoPoint("");// todo scale to multiple drones
        home_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(home_geo_point_);
    }
    catch (rpc::rpc_error&  e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimROSWrapper::initialize_ros()
{
    nh_ = getNodeHandle();
    nh_private_ = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh_);

    // ros params
    vel_cmd_duration_ = 0.05; // todo
    double update_airsim_img_response_every_n_sec;// = 0.0001;
    double update_airsim_control_every_n_sec;// = 0.01;

    // todo enforce dynamics constraints in this node as well?
    // nh_.getParam("max_vert_vel_", max_vert_vel_);
    // nh_.getParam("max_horz_vel", max_horz_vel_);

    nh_private_.getParam("/cameras", camera_name_image_type_list_);
    generate_img_request_vec_and_ros_pubs_from_sensors_yml();

    nh_private_.getParam("is_vulkan", is_vulkan_);
    nh_private_.getParam("front_left_calib_file", front_left_calib_file_);
    nh_private_.getParam("front_right_calib_file", front_right_calib_file_);

    nh_private_.getParam("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
    nh_private_.getParam("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);

    // fill camera info msg from YAML calib file. todo error check path
    read_params_from_yaml_and_fill_cam_info_msg(front_left_calib_file_, front_left_cam_info_msg_);
    front_left_cam_info_msg_.header.frame_id = "airsim/front/left";

    read_params_from_yaml_and_fill_cam_info_msg(front_right_calib_file_, front_right_cam_info_msg_);
    front_right_cam_info_msg_.header.frame_id = "airsim/front/right";

    takeoff_srvr_ = nh_private_.advertiseService("takeoff", &AirsimROSWrapper::takeoff_srv_cb, this);
    land_srvr_ = nh_private_.advertiseService("land", &AirsimROSWrapper::land_srv_cb, this);
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimROSWrapper::reset_srv_cb, this);

    // clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 10); // mimic gazebo's /use_sim_time feature
    vehicle_state_pub_ = nh_private_.advertise<mavros_msgs::State>("vehicle_state", 10);
    odom_local_ned_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_local_ned", 10);
    global_gps_pub_ = nh_private_.advertise<sensor_msgs::NavSatFix>("global_gps", 10);
    home_geo_point_pub_ = nh_private_.advertise<airsim_ros_pkgs::GPSYaw>("home_geo_point", 10);
    imu_ground_truth_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu_ground_truth", 10);

    // subscribe to control commands on global nodehandle
    vel_cmd_body_frame_sub_ = nh_.subscribe("vel_cmd_body_frame", 50, &AirsimROSWrapper::vel_cmd_body_frame_cb, this); // todo ros::TransportHints().tcpNoDelay();
    vel_cmd_world_frame_sub_ = nh_.subscribe("vel_cmd_world_frame", 50, &AirsimROSWrapper::vel_cmd_world_frame_cb, this);
    gimbal_angle_quat_cmd_sub_ = nh_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);
    gimbal_angle_euler_cmd_sub_ = nh_.subscribe("gimbal_angle_euler_cmd", 50, &AirsimROSWrapper::gimbal_angle_euler_cmd_cb, this);

    airsim_img_response_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_img_response_every_n_sec), &AirsimROSWrapper::img_response_timer_cb, this);
    airsim_control_update_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimROSWrapper::drone_state_timer_cb, this);
}

// todo minor: error check. if state is not landed, return error. 
bool AirsimROSWrapper::takeoff_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.takeoffAsync()->waitOnLastTask();
    in_air_ = true;
    return true; //todo
}

// todo minor: error check. if state is not in air, return error. 
bool AirsimROSWrapper::land_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.landAsync()->waitOnLastTask();
    in_air_ = false;
    return true; //todo
}

bool AirsimROSWrapper::reset_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.reset();
    return true; //todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat)
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat)
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z); 
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat)
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z()); 
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd &msg)
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(curr_drone_state_.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw

    // todo do actual body frame?
    vel_cmd_.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    vel_cmd_.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw)); //body frame
    vel_cmd_.z = msg.twist.linear.z;
    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd_.yaw_mode.is_rate = true;
    // airsim uses degrees
    vel_cmd_.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    vel_cmd_.vehicle_name = msg.vehicle_name;
    has_vel_cmd_ = true;
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd &msg)
{
    vel_cmd_.x = msg.twist.linear.x;
    vel_cmd_.y = msg.twist.linear.y;
    vel_cmd_.z = msg.twist.linear.z;
    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd_.yaw_mode.is_rate = true;
    vel_cmd_.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    vel_cmd_.vehicle_name = msg.vehicle_name;
    has_vel_cmd_ = true;
}

// todo support multiple gimbal commands
void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd &gimbal_angle_quat_cmd_msg)
{
    tf2::Quaternion quat_control_cmd;
    try
    {
        tf2::convert(gimbal_angle_quat_cmd_msg.orientation, quat_control_cmd);
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd); // airsim uses wxyz
        gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg.camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg.vehicle_name;
        has_gimbal_cmd_ = true; 
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

// todo support multiple gimbal commands
// todo make transforms name class members and ros params. make an unordered map
// 1. find quaternion of default gimbal pose
// 2. forward multiply with quaternion equivalent to desired euler commands (in degrees)
// 3. call airsim client's setcameraorientation which sets camera orientation wrt world (or takeoff?) ned frame. todo 
void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd &gimbal_angle_euler_cmd_msg)
{
    try
    {
        tf2::Quaternion quat_control_cmd;
        quat_control_cmd.setRPY(math_common::deg2rad(gimbal_angle_euler_cmd_msg.roll), math_common::deg2rad(gimbal_angle_euler_cmd_msg.pitch), math_common::deg2rad(gimbal_angle_euler_cmd_msg.yaw));
        quat_control_cmd.normalize();
        gimbal_cmd_.target_quat = get_airlib_quat(quat_control_cmd);
        gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg.camera_name;
        gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg.vehicle_name;
        has_gimbal_cmd_ = true; 
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
}

// todo to pass param and fill, or return value. 
nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state)
{
    nav_msgs::Odometry odom_ned_msg;
    // odom_ned_msg.header.frame = ;
    odom_ned_msg.child_frame_id = "/airsim/odom_local_ned"; // todo make param

    odom_ned_msg.pose.pose.position.x = drone_state.getPosition().x();
    odom_ned_msg.pose.pose.position.y = drone_state.getPosition().y();
    odom_ned_msg.pose.pose.position.z = drone_state.getPosition().z();
    odom_ned_msg.pose.pose.orientation.x = drone_state.getOrientation().x();
    odom_ned_msg.pose.pose.orientation.y = drone_state.getOrientation().y();
    odom_ned_msg.pose.pose.orientation.z = drone_state.getOrientation().z();
    odom_ned_msg.pose.pose.orientation.w = drone_state.getOrientation().w();

    odom_ned_msg.twist.twist.linear.x = drone_state.kinematics_estimated.twist.linear.x();
    odom_ned_msg.twist.twist.linear.y = drone_state.kinematics_estimated.twist.linear.y();
    odom_ned_msg.twist.twist.linear.z = drone_state.kinematics_estimated.twist.linear.z();
    odom_ned_msg.twist.twist.angular.x = drone_state.kinematics_estimated.twist.angular.x();
    odom_ned_msg.twist.twist.angular.y = drone_state.kinematics_estimated.twist.angular.y();
    odom_ned_msg.twist.twist.angular.z = drone_state.kinematics_estimated.twist.angular.z();

    return odom_ned_msg;
}

// todo covariances
sensor_msgs::Imu AirsimROSWrapper::get_ground_truth_imu_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state)
{
    sensor_msgs::Imu imu_msg;
    // imu_msg.header.frame = ;
    imu_msg.orientation.x = drone_state.getOrientation().x();
    imu_msg.orientation.y = drone_state.getOrientation().y();
    imu_msg.orientation.z = drone_state.getOrientation().z();
    imu_msg.orientation.w = drone_state.getOrientation().w();

    // imu_msg.orientation_covariance = ;

    // todo radians per second
    imu_msg.angular_velocity.x = math_common::deg2rad(drone_state.kinematics_estimated.twist.angular.x());
    imu_msg.angular_velocity.y = math_common::deg2rad(drone_state.kinematics_estimated.twist.angular.y());
    imu_msg.angular_velocity.z = math_common::deg2rad(drone_state.kinematics_estimated.twist.angular.z());

    // imu_msg.angular_velocity_covariance = ;

    // meters/s2^m 
    // todo const struct msr::airlib::Kinematics::State’ has no member named ‘linear_acceleration
    // imu_msg.linear_acceleration.x = drone_state.kinematics_estimated.linear_acceleration.x();
    // imu_msg.linear_acceleration.y = drone_state.kinematics_estimated.linear_acceleration.y();
    // imu_msg.linear_acceleration.z = drone_state.kinematics_estimated.linear_acceleration.z();

    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}


void AirsimROSWrapper::publish_odom_tf(const nav_msgs::Odometry &odom_ned_msg)
{
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_ned_msg.header;
    odom_tf.child_frame_id = odom_ned_msg.child_frame_id; 
    odom_tf.transform.translation.x = odom_ned_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_ned_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_ned_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_ned_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_ned_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_ned_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_ned_msg.pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(odom_tf);
}

airsim_ros_pkgs::GPSYaw AirsimROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint &geo_point)
{
    airsim_ros_pkgs::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

sensor_msgs::NavSatFix AirsimROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint &geo_point)
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}

mavros_msgs::State AirsimROSWrapper::get_vehicle_state_msg(msr::airlib::MultirotorState &drone_state)
{
    mavros_msgs::State vehicle_state_msg;
    // vehicle_state_msg.connected = true; // not reqd
    vehicle_state_msg.armed = true; // todo is_armed_
    // vehicle_state_msg.guided; // not reqd 
    // vehicle_state_msg.mode; // todo
    // vehicle_state_msg.system_status; // not reqd
    return vehicle_state_msg;
}

// todo unused
void AirsimROSWrapper::set_zero_vel_cmd()
{
    vel_cmd_.x = 0.0;
    vel_cmd_.y = 0.0;
    vel_cmd_.z = 0.0;

    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd_.yaw_mode.is_rate = false;

    // todo make class member or a fucntion 
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(curr_drone_state_.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw
    vel_cmd_.yaw_mode.yaw_or_rate = yaw;
}

void AirsimROSWrapper::drone_state_timer_cb(const ros::TimerEvent& event)
{
    // get drone state from airsim
    curr_drone_state_ = airsim_client_.getMultirotorState();
    ros::Time curr_ros_time = ros::Time::now();

    // convert airsim drone state to ROS msgs
    curr_odom_ned_ = get_odom_msg_from_airsim_state(curr_drone_state_);
    curr_odom_ned_.header.frame_id = "world_ned";
    curr_odom_ned_.header.stamp = curr_ros_time;

    sensor_msgs::NavSatFix gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(curr_drone_state_.gps_location);
    gps_sensor_msg.header.stamp = curr_ros_time;

    mavros_msgs::State vehicle_state_msg = get_vehicle_state_msg(curr_drone_state_);

    // sensor_msgs::Imu imu_ground_truth_msg = get_ground_truth_imu_msg_from_airsim_state(curr_drone_state_);

    // publish to ROS!  
    odom_local_ned_pub_.publish(curr_odom_ned_);
    publish_odom_tf(curr_odom_ned_);
    global_gps_pub_.publish(gps_sensor_msg);
    home_geo_point_pub_.publish(home_geo_point_msg_);
    vehicle_state_pub_.publish(vehicle_state_msg);
    // imu_ground_truth_pub_.publish(imu_ground_truth_msg);//todo. IMU is pretty fast. should be in its own timer callback 

    // send control commands from the last callback to airsim
    if (has_vel_cmd_)
        airsim_client_.moveByVelocityAsync(vel_cmd_.x, vel_cmd_.y, vel_cmd_.z, vel_cmd_duration_, 
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, vel_cmd_.yaw_mode);

    // todo add and expose a gimbal angular velocity to airlib
    if (has_gimbal_cmd_)
        airsim_client_.simSetCameraOrientation(gimbal_cmd_.camera_name, gimbal_cmd_.target_quat, gimbal_cmd_.vehicle_name);

    // "clear" control cmds
    has_vel_cmd_ = false;
    has_gimbal_cmd_ = false;
}

// XmlRpc::XmlRpcValue can't be const in this case
void AirsimROSWrapper::generate_img_request_vec_and_ros_pubs_from_sensors_yml()
{
    airsim_img_request_.clear();
    image_pub_vec_.clear();
    image_types_names_vec_.clear();
    cam_info_pub_vec_.clear();

    image_transport::ImageTransport image_transporter(nh_);

    // iterate over request cameras
    for (auto& it : camera_name_image_type_list_)
    {
        std::string curr_camera_name = std::string(it.first);
        cam_name_to_cam_tf_name_map_[curr_camera_name] = curr_camera_name + "_optical"; // tf name

        // iterate over request capture types
        for (int image_type_idx = 0; image_type_idx < it.second.size(); image_type_idx++)
        {
            std::cout << "Initializing ROS publisher for camera name \"" << it.first << 
                "\" with image type \"" << it.second[image_type_idx] << "\" " << std::endl;

            // int image types 
            if (it.second[image_type_idx] == "Scene" || 
                it.second[image_type_idx] == "Segmentation" || 
                it.second[image_type_idx] == "SurfaceNormals" || 
                it.second[image_type_idx] == "Infrared")
            {
                // append to ImageRequest to be sent to image
                airsim_img_request_.push_back(ImageRequest(curr_camera_name, ImageType::Scene, false, false));
            }
            else // float image types DepthPlanner || DepthPerspective || DepthVis || DisparityNormalized
            {
                airsim_img_request_.push_back(ImageRequest(curr_camera_name, ImageType::DepthPlanner, true));
            }
            // append a corresponding ROS publisher
            image_pub_vec_.push_back(image_transporter.advertise(curr_camera_name + "/" + std::string(it.second[image_type_idx]), 1));
            cam_info_pub_vec_.push_back(nh_.advertise<sensor_msgs::CameraInfo> (curr_camera_name + "/" + std::string(it.second[image_type_idx]) + "/camera_info", 10));
            image_types_names_vec_.push_back(std::string(it.second[image_type_idx]));
        }
    }
}

// the image request names should match the json custom camera names!
void AirsimROSWrapper::img_response_timer_cb(const ros::TimerEvent& event)
{    
    try
    {
        const std::vector<ImageResponse>& img_response = airsim_client_.simGetImages(airsim_img_request_);

        if (img_response.size() == airsim_img_request_.size()) 
        {
            process_and_publish_img_response(img_response);
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}

cv::Mat AirsimROSWrapper::manual_decode_depth(const ImageResponse &img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
    return mat;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_img_msg_from_response(const ImageResponse& img_response,
                                                                const ros::Time curr_ros_time, 
                                                                const std::string frame_id)
{
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = curr_ros_time;
    img_msg_ptr->header.frame_id = frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "rgb8";
    if (is_vulkan_)
        img_msg_ptr->encoding = "bgr8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr AirsimROSWrapper::get_depth_img_msg_from_response(const ImageResponse& img_response,
                                                                        const ros::Time curr_ros_time,
                                                                        const std::string frame_id)
{
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error, 
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = manual_decode_depth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = curr_ros_time;
    depth_img_msg->header.frame_id = frame_id;
    return depth_img_msg;
}



sensor_msgs::CameraInfo AirsimROSWrapper::get_cam_info_msg(const ImageResponse& img_response,
                                                            float fov_degrees,
                                                            const ros::Time curr_ros_time,
                                                            const std::string frame_id)
{
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.stamp = curr_ros_time;
    cam_info_msg.header.frame_id = frame_id;
    cam_info_msg.height = img_response.height;
    cam_info_msg.width = img_response.width;
    float f_x = (img_response.width / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    // focal length in Y direction should be same as X it seems. 
    // float f_y = (img_response.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = {f_x, 0.0, img_response.width / 2.0, 
                        0.0, f_x, img_response.height / 2.0, 
                        0.0, 0.0, 1.0};
    cam_info_msg.P = {f_x, 0.0, img_response.width / 2.0, 0.0,
                        0.0, f_x, img_response.height / 2.0, 0.0, 
                        0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec)
{    
    ros::Time curr_ros_time = ros::Time::now(); // todo add option to use airsim time (image_response.TTimePoint) like Gazebo /use_sim_time param
    int img_response_idx = 0;

    for (const auto& curr_img_response : img_response_vec)
    {
        // todo publishing a tf for each capture type seems stupid. but it foolproofs us against render thread's async stuff, I hope. 
        // Ideally, we should loop over cameras and then captures, and publish only one tf.  
        // publish_camera_tf(curr_img_response, curr_ros_time, "world_ned", curr_img_response.camera_name + "/" + image_types_names_vec_[img_response_idx]);
        publish_camera_tf(curr_img_response, curr_ros_time, "world_ned", curr_img_response.camera_name);

        // todoo calling airsim_client_ in this funciton isn'it the cleanest thing to do. should be in img_response_timer_cb for readability  
        msr::airlib::CameraInfo camera_info = airsim_client_.simGetCameraInfo(curr_img_response.camera_name);
        // std::cout << curr_img_response.camera_name << ", " << camera_info.fov << std::endl;

        cam_info_pub_vec_[img_response_idx].publish(get_cam_info_msg(curr_img_response, camera_info.fov, curr_ros_time, 
            curr_img_response.camera_name + "/" + image_types_names_vec_[img_response_idx] + "_optical"));
        if (curr_img_response.pixels_as_float) // DepthPlanner / DepthPerspective / DepthVis / DisparityNormalized
        {
            image_pub_vec_[img_response_idx].publish(get_depth_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical"));
                                                    // curr_img_response.camera_name + "/" + image_types_names_vec_[img_response_idx] + "_optical"));
        }
        else // Scene / Segmentation / SurfaceNormals / Infrared
        {
            image_pub_vec_[img_response_idx].publish(get_img_msg_from_response(curr_img_response, 
                                                    curr_ros_time, 
                                                    curr_img_response.camera_name + "_optical"));
                                                    // curr_img_response.camera_name + "/" + image_types_names_vec_[img_response_idx] + "_optical"));
        }
        img_response_idx++;
    }

    // update timestamp of saved cam info msgs
}

// publish camera transforms
// camera poses are obtained from airsim's client API which are in (local) NED frame. 
// We first do a change of basis to camera optical frame (Z forward, X right, Y down) 
void AirsimROSWrapper::publish_camera_tf(const ImageResponse &img_response, const ros::Time &ros_time, const std::string &frame_id, const std::string &child_frame_id)
{
    geometry_msgs::TransformStamped cam_tf_body_msg;
    cam_tf_body_msg.header.stamp = ros_time;
    cam_tf_body_msg.header.frame_id = frame_id;
    cam_tf_body_msg.child_frame_id = child_frame_id;
    cam_tf_body_msg.transform.translation.x = img_response.camera_position.x();
    cam_tf_body_msg.transform.translation.y = img_response.camera_position.y();
    cam_tf_body_msg.transform.translation.z = img_response.camera_position.z();
    cam_tf_body_msg.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf_body_msg.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf_body_msg.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf_body_msg.transform.rotation.w = img_response.camera_orientation.w();

    geometry_msgs::TransformStamped cam_tf_optical_msg;
    cam_tf_optical_msg.header.stamp = ros_time;
    cam_tf_optical_msg.header.frame_id = frame_id;
    cam_tf_optical_msg.child_frame_id = child_frame_id + "_optical";
    cam_tf_optical_msg.transform.translation.x = cam_tf_body_msg.transform.translation.x;
    cam_tf_optical_msg.transform.translation.y = cam_tf_body_msg.transform.translation.y;
    cam_tf_optical_msg.transform.translation.z = cam_tf_body_msg.transform.translation.z;

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(cam_tf_body_msg.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body); 
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body * matrix_cam_body_to_optical_inverse_;
    // tf2::Matrix3x3 mat_cam_optical = matrix_cam_body_to_optical_ * mat_cam_body;
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(), 
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ()); 
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, cam_tf_optical_msg.transform.rotation);

    tf_broadcaster_.sendTransform(cam_tf_body_msg);
    tf_broadcaster_.sendTransform(cam_tf_optical_msg);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}