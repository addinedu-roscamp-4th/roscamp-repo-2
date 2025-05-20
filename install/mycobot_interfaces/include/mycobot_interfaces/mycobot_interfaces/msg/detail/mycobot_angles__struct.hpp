// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mycobot_interfaces:msg/MycobotAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_angles.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mycobot_interfaces__msg__MycobotAngles __attribute__((deprecated))
#else
# define DEPRECATED__mycobot_interfaces__msg__MycobotAngles __declspec(deprecated)
#endif

namespace mycobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MycobotAngles_
{
  using Type = MycobotAngles_<ContainerAllocator>;

  explicit MycobotAngles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_1 = 0.0f;
      this->joint_2 = 0.0f;
      this->joint_3 = 0.0f;
      this->joint_4 = 0.0f;
      this->joint_5 = 0.0f;
      this->joint_6 = 0.0f;
    }
  }

  explicit MycobotAngles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_1 = 0.0f;
      this->joint_2 = 0.0f;
      this->joint_3 = 0.0f;
      this->joint_4 = 0.0f;
      this->joint_5 = 0.0f;
      this->joint_6 = 0.0f;
    }
  }

  // field types and members
  using _joint_1_type =
    float;
  _joint_1_type joint_1;
  using _joint_2_type =
    float;
  _joint_2_type joint_2;
  using _joint_3_type =
    float;
  _joint_3_type joint_3;
  using _joint_4_type =
    float;
  _joint_4_type joint_4;
  using _joint_5_type =
    float;
  _joint_5_type joint_5;
  using _joint_6_type =
    float;
  _joint_6_type joint_6;

  // setters for named parameter idiom
  Type & set__joint_1(
    const float & _arg)
  {
    this->joint_1 = _arg;
    return *this;
  }
  Type & set__joint_2(
    const float & _arg)
  {
    this->joint_2 = _arg;
    return *this;
  }
  Type & set__joint_3(
    const float & _arg)
  {
    this->joint_3 = _arg;
    return *this;
  }
  Type & set__joint_4(
    const float & _arg)
  {
    this->joint_4 = _arg;
    return *this;
  }
  Type & set__joint_5(
    const float & _arg)
  {
    this->joint_5 = _arg;
    return *this;
  }
  Type & set__joint_6(
    const float & _arg)
  {
    this->joint_6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> *;
  using ConstRawPtr =
    const mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mycobot_interfaces__msg__MycobotAngles
    std::shared_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mycobot_interfaces__msg__MycobotAngles
    std::shared_ptr<mycobot_interfaces::msg::MycobotAngles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MycobotAngles_ & other) const
  {
    if (this->joint_1 != other.joint_1) {
      return false;
    }
    if (this->joint_2 != other.joint_2) {
      return false;
    }
    if (this->joint_3 != other.joint_3) {
      return false;
    }
    if (this->joint_4 != other.joint_4) {
      return false;
    }
    if (this->joint_5 != other.joint_5) {
      return false;
    }
    if (this->joint_6 != other.joint_6) {
      return false;
    }
    return true;
  }
  bool operator!=(const MycobotAngles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MycobotAngles_

// alias to use template instance with default allocator
using MycobotAngles =
  mycobot_interfaces::msg::MycobotAngles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__STRUCT_HPP_
