// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/cook_state.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mycobot_interfaces__msg__CookState __attribute__((deprecated))
#else
# define DEPRECATED__mycobot_interfaces__msg__CookState __declspec(deprecated)
#endif

namespace mycobot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CookState_
{
  using Type = CookState_<ContainerAllocator>;

  explicit CookState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->order_id = 0l;
    }
  }

  explicit CookState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->order_id = 0l;
    }
  }

  // field types and members
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _order_id_type =
    int32_t;
  _order_id_type order_id;

  // setters for named parameter idiom
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__order_id(
    const int32_t & _arg)
  {
    this->order_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mycobot_interfaces::msg::CookState_<ContainerAllocator> *;
  using ConstRawPtr =
    const mycobot_interfaces::msg::CookState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::CookState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mycobot_interfaces::msg::CookState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mycobot_interfaces__msg__CookState
    std::shared_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mycobot_interfaces__msg__CookState
    std::shared_ptr<mycobot_interfaces::msg::CookState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CookState_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->order_id != other.order_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const CookState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CookState_

// alias to use template instance with default allocator
using CookState =
  mycobot_interfaces::msg::CookState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__STRUCT_HPP_
