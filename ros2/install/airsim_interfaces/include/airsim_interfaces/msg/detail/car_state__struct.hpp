// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__CarState __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__CarState __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarState_
{
  using Type = CarState_<ContainerAllocator>;

  explicit CarState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->gear = 0;
      this->rpm = 0.0f;
      this->maxrpm = 0.0f;
      this->handbrake = false;
    }
  }

  explicit CarState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->gear = 0;
      this->rpm = 0.0f;
      this->maxrpm = 0.0f;
      this->handbrake = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _twist_type twist;
  using _speed_type =
    float;
  _speed_type speed;
  using _gear_type =
    int8_t;
  _gear_type gear;
  using _rpm_type =
    float;
  _rpm_type rpm;
  using _maxrpm_type =
    float;
  _maxrpm_type maxrpm;
  using _handbrake_type =
    bool;
  _handbrake_type handbrake;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__gear(
    const int8_t & _arg)
  {
    this->gear = _arg;
    return *this;
  }
  Type & set__rpm(
    const float & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__maxrpm(
    const float & _arg)
  {
    this->maxrpm = _arg;
    return *this;
  }
  Type & set__handbrake(
    const bool & _arg)
  {
    this->handbrake = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::CarState_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::CarState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::CarState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::CarState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__CarState
    std::shared_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__CarState
    std::shared_ptr<airsim_interfaces::msg::CarState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->gear != other.gear) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->maxrpm != other.maxrpm) {
      return false;
    }
    if (this->handbrake != other.handbrake) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarState_

// alias to use template instance with default allocator
using CarState =
  airsim_interfaces::msg::CarState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__CAR_STATE__STRUCT_HPP_
