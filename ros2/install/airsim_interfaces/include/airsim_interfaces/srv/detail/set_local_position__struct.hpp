// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from airsim_interfaces:srv/SetLocalPosition.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_HPP_
#define AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Request __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLocalPosition_Request_
{
  using Type = SetLocalPosition_Request_<ContainerAllocator>;

  explicit SetLocalPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->yaw = 0.0;
      this->vehicle_name = "";
    }
  }

  explicit SetLocalPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vehicle_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->yaw = 0.0;
      this->vehicle_name = "";
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _vehicle_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _vehicle_name_type vehicle_name;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
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
    airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Request
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Request
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLocalPosition_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
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
  bool operator!=(const SetLocalPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLocalPosition_Request_

// alias to use template instance with default allocator
using SetLocalPosition_Request =
  airsim_interfaces::srv::SetLocalPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces


#ifndef _WIN32
# define DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Response __declspec(deprecated)
#endif

namespace airsim_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLocalPosition_Response_
{
  using Type = SetLocalPosition_Response_<ContainerAllocator>;

  explicit SetLocalPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetLocalPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Response
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__airsim_interfaces__srv__SetLocalPosition_Response
    std::shared_ptr<airsim_interfaces::srv::SetLocalPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLocalPosition_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLocalPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLocalPosition_Response_

// alias to use template instance with default allocator
using SetLocalPosition_Response =
  airsim_interfaces::srv::SetLocalPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace airsim_interfaces

namespace airsim_interfaces
{

namespace srv
{

struct SetLocalPosition
{
  using Request = airsim_interfaces::srv::SetLocalPosition_Request;
  using Response = airsim_interfaces::srv::SetLocalPosition_Response;
};

}  // namespace srv

}  // namespace airsim_interfaces

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__SET_LOCAL_POSITION__STRUCT_HPP_
