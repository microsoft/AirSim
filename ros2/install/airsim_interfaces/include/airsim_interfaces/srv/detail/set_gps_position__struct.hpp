// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:srv/SetGPSPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__STRUCT_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Request __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGPSPosition_Request_
{
  using Type = SetGPSPosition_Request_<ContainerAllocator>;

  explicit SetGPSPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->yaw = 0.0;
      this->vehicle_name = "";
    }
  }

  explicit SetGPSPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vehicle_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->yaw = 0.0;
      this->vehicle_name = "";
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _vehicle_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _vehicle_name_type vehicle_name;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__vehicle_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->vehicle_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Request
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Request
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGPSPosition_Request_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->vehicle_name != other.vehicle_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGPSPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGPSPosition_Request_

// alias to use template instance with default allocator
using SetGPSPosition_Request =
  airsim_interfaces::srv::SetGPSPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Response __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGPSPosition_Response_
{
  using Type = SetGPSPosition_Response_<ContainerAllocator>;

  explicit SetGPSPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetGPSPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Response
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__SetGPSPosition_Response
    std::shared_ptr<airsim_interfaces::srv::SetGPSPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGPSPosition_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGPSPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGPSPosition_Response_

// alias to use template instance with default allocator
using SetGPSPosition_Response =
  airsim_interfaces::srv::SetGPSPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces

namespace airsim_interfaces
{

namespace srv
{

struct SetGPSPosition
{
  using Request = airsim_interfaces::srv::SetGPSPosition_Request;
  using Response = airsim_interfaces::srv::SetGPSPosition_Response;
};

}  // namespace srv

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_GPS_POSITION__STRUCT_HPP_
