// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_GimbalAngleEulerCmd_yaw
{
public:
  explicit Init_GimbalAngleEulerCmd_yaw(::airsim_interfaces::msg::GimbalAngleEulerCmd & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::GimbalAngleEulerCmd yaw(::airsim_interfaces::msg::GimbalAngleEulerCmd::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

class Init_GimbalAngleEulerCmd_pitch
{
public:
  explicit Init_GimbalAngleEulerCmd_pitch(::airsim_interfaces::msg::GimbalAngleEulerCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleEulerCmd_yaw pitch(::airsim_interfaces::msg::GimbalAngleEulerCmd::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GimbalAngleEulerCmd_yaw(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

class Init_GimbalAngleEulerCmd_roll
{
public:
  explicit Init_GimbalAngleEulerCmd_roll(::airsim_interfaces::msg::GimbalAngleEulerCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleEulerCmd_pitch roll(::airsim_interfaces::msg::GimbalAngleEulerCmd::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_GimbalAngleEulerCmd_pitch(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

class Init_GimbalAngleEulerCmd_vehicle_name
{
public:
  explicit Init_GimbalAngleEulerCmd_vehicle_name(::airsim_interfaces::msg::GimbalAngleEulerCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleEulerCmd_roll vehicle_name(::airsim_interfaces::msg::GimbalAngleEulerCmd::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return Init_GimbalAngleEulerCmd_roll(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

class Init_GimbalAngleEulerCmd_camera_name
{
public:
  explicit Init_GimbalAngleEulerCmd_camera_name(::airsim_interfaces::msg::GimbalAngleEulerCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalAngleEulerCmd_vehicle_name camera_name(::airsim_interfaces::msg::GimbalAngleEulerCmd::_camera_name_type arg)
  {
    msg_.camera_name = std::move(arg);
    return Init_GimbalAngleEulerCmd_vehicle_name(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

class Init_GimbalAngleEulerCmd_header
{
public:
  Init_GimbalAngleEulerCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalAngleEulerCmd_camera_name header(::airsim_interfaces::msg::GimbalAngleEulerCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalAngleEulerCmd_camera_name(msg_);
  }

private:
  ::airsim_interfaces::msg::GimbalAngleEulerCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::GimbalAngleEulerCmd>()
{
  return airsim_interfaces::msg::builder::Init_GimbalAngleEulerCmd_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__BUILDER_HPP_
