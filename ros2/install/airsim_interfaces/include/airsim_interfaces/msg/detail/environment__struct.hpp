// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_
#define AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_

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
// Member 'position'
// Member 'gravity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'geo_point'
#include "geographic_msgs/msg/detail/geo_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__msg__Environment __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__msg__Environment __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Environment_
{
  using Type = Environment_<ContainerAllocator>;

  explicit Environment_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    geo_point(_init),
    gravity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->air_pressure = 0.0f;
      this->temperature = 0.0f;
      this->air_density = 0.0f;
    }
  }

  explicit Environment_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    geo_point(_alloc, _init),
    gravity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->air_pressure = 0.0f;
      this->temperature = 0.0f;
      this->air_density = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_type position;
  using _geo_point_type =
    geographic_msgs::msg::GeoPoint_<ContainerAllocator>;
  _geo_point_type geo_point;
  using _gravity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gravity_type gravity;
  using _air_pressure_type =
    float;
  _air_pressure_type air_pressure;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _air_density_type =
    float;
  _air_density_type air_density;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__geo_point(
    const geographic_msgs::msg::GeoPoint_<ContainerAllocator> & _arg)
  {
    this->geo_point = _arg;
    return *this;
  }
  Type & set__gravity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gravity = _arg;
    return *this;
  }
  Type & set__air_pressure(
    const float & _arg)
  {
    this->air_pressure = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__air_density(
    const float & _arg)
  {
    this->air_density = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::msg::Environment_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::msg::Environment_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::Environment_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::msg::Environment_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__msg__Environment
    std::shared_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__msg__Environment
    std::shared_ptr<airsim_interfaces::msg::Environment_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Environment_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->geo_point != other.geo_point) {
      return false;
    }
    if (this->gravity != other.gravity) {
      return false;
    }
    if (this->air_pressure != other.air_pressure) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->air_density != other.air_density) {
      return false;
    }
    return true;
  }
  bool operator!=(const Environment_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Environment_

// alias to use template instance with default allocator
using Environment =
  airsim_interfaces::msg::Environment_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_
