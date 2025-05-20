// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mycobot_interfaces:msg/MycobotCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_coords.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_COORDS__STRUCT_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_COORDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mycobot_interfaces__msg__MycobotCoords __attribute__((deprecated))
#else
# define DEPRECATED__mycobot_interfaces__msg__MycobotCoords __declspec(deprecated)
#endif

namespace mycobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MycobotCoords_
{
  using Type = MycobotCoords_<ContainerAllocator>;

  explicit MycobotCoords_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
      this->rz = 0.0f;
    }
  }

  explicit MycobotCoords_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->rx = 0.0f;
      this->ry = 0.0f;
      this->rz = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _rx_type =
    float;
  _rx_type rx;
  using _ry_type =
    float;
  _ry_type ry;
  using _rz_type =
    float;
  _rz_type rz;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__rx(
    const float & _arg)
  {
    this->rx = _arg;
    return *this;
  }
  Type & set__ry(
    const float & _arg)
  {
    this->ry = _arg;
    return *this;
  }
  Type & set__rz(
    const float & _arg)
  {
    this->rz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> *;
  using ConstRawPtr =
    const mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mycobot_interfaces__msg__MycobotCoords
    std::shared_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mycobot_interfaces__msg__MycobotCoords
    std::shared_ptr<mycobot_interfaces::msg::MycobotCoords_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MycobotCoords_ & other) const
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
    if (this->rx != other.rx) {
      return false;
    }
    if (this->ry != other.ry) {
      return false;
    }
    if (this->rz != other.rz) {
      return false;
    }
    return true;
  }
  bool operator!=(const MycobotCoords_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MycobotCoords_

// alias to use template instance with default allocator
using MycobotCoords =
  mycobot_interfaces::msg::MycobotCoords_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_COORDS__STRUCT_HPP_
