// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/cook_state.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__BUILDER_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/msg/detail/cook_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CookState_order_id
{
public:
  explicit Init_CookState_order_id(::mycobot_interfaces::msg::CookState & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::msg::CookState order_id(::mycobot_interfaces::msg::CookState::_order_id_type arg)
  {
    msg_.order_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::msg::CookState msg_;
};

class Init_CookState_state
{
public:
  Init_CookState_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CookState_order_id state(::mycobot_interfaces::msg::CookState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CookState_order_id(msg_);
  }

private:
  ::mycobot_interfaces::msg::CookState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::msg::CookState>()
{
  return mycobot_interfaces::msg::builder::Init_CookState_state();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__BUILDER_HPP_
