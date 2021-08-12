// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__GimbalAngleEulerCmd __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__GimbalAngleEulerCmd __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalAngleEulerCmd_
{
  using Type = GimbalAngleEulerCmd_<ContainerAllocator>;

  explicit GimbalAngleEulerCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->camera_name = "";
      this->vehicle_name = "";
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit GimbalAngleEulerCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    camera_name(_alloc),
    vehicle_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->camera_name = "";
      this->vehicle_name = "";
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _camera_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _camera_name_type camera_name;
  using _vehicle_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _vehicle_name_type vehicle_name;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__camera_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->camera_name = _arg;
    return *this;
  }
  Type & set__vehicle_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->vehicle_name = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__GimbalAngleEulerCmd
    std::shared_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__GimbalAngleEulerCmd
    std::shared_ptr<airsim_interfaces::msg::GimbalAngleEulerCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalAngleEulerCmd_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->camera_name != other.camera_name) {
      return false;
    }
    if (this->vehicle_name != other.vehicle_name) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalAngleEulerCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalAngleEulerCmd_

// alias to use template instance with default allocator
using GimbalAngleEulerCmd =
  airsim_interfaces::msg::GimbalAngleEulerCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__STRUCT_HPP_
