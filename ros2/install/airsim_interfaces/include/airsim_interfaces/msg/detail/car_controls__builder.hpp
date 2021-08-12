// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from airsim_interfaces:msg/CarControls.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__BUILDER_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__BUILDER_HPP_

#include "airsim_interfaces/msg/detail/car_controls__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace airsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_CarControls_gear_immediate
{
public:
  explicit Init_CarControls_gear_immediate(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  ::airsim_interfaces::msg::CarControls gear_immediate(::airsim_interfaces::msg::CarControls::_gear_immediate_type arg)
  {
    msg_.gear_immediate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_manual_gear
{
public:
  explicit Init_CarControls_manual_gear(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_gear_immediate manual_gear(::airsim_interfaces::msg::CarControls::_manual_gear_type arg)
  {
    msg_.manual_gear = std::move(arg);
    return Init_CarControls_gear_immediate(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_manual
{
public:
  explicit Init_CarControls_manual(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_manual_gear manual(::airsim_interfaces::msg::CarControls::_manual_type arg)
  {
    msg_.manual = std::move(arg);
    return Init_CarControls_manual_gear(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_handbrake
{
public:
  explicit Init_CarControls_handbrake(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_manual handbrake(::airsim_interfaces::msg::CarControls::_handbrake_type arg)
  {
    msg_.handbrake = std::move(arg);
    return Init_CarControls_manual(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_steering
{
public:
  explicit Init_CarControls_steering(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_handbrake steering(::airsim_interfaces::msg::CarControls::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_CarControls_handbrake(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_brake
{
public:
  explicit Init_CarControls_brake(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_steering brake(::airsim_interfaces::msg::CarControls::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_CarControls_steering(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_throttle
{
public:
  explicit Init_CarControls_throttle(::airsim_interfaces::msg::CarControls & msg)
  : msg_(msg)
  {}
  Init_CarControls_brake throttle(::airsim_interfaces::msg::CarControls::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_CarControls_brake(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

class Init_CarControls_header
{
public:
  Init_CarControls_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarControls_throttle header(::airsim_interfaces::msg::CarControls::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CarControls_throttle(msg_);
  }

private:
  ::airsim_interfaces::msg::CarControls msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::airsim_interfaces::msg::CarControls>()
{
  return airsim_interfaces::msg::builder::Init_CarControls_header();
}

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_CONTROLS__BUILDER_HPP_
