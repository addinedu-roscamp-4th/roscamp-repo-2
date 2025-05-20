// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:msg/MycobotPumpStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_pump_status.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/msg/detail/mycobot_pump_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MycobotPumpStatus_pin2
{
public:
  explicit Init_MycobotPumpStatus_pin2(::mycobot_interfaces::msg::MycobotPumpStatus & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::msg::MycobotPumpStatus pin2(::mycobot_interfaces::msg::MycobotPumpStatus::_pin2_type arg)
  {
    msg_.pin2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotPumpStatus msg_;
};

class Init_MycobotPumpStatus_pin1
{
public:
  explicit Init_MycobotPumpStatus_pin1(::mycobot_interfaces::msg::MycobotPumpStatus & msg)
  : msg_(msg)
  {}
  Init_MycobotPumpStatus_pin2 pin1(::mycobot_interfaces::msg::MycobotPumpStatus::_pin1_type arg)
  {
    msg_.pin1 = std::move(arg);
    return Init_MycobotPumpStatus_pin2(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotPumpStatus msg_;
};

class Init_MycobotPumpStatus_status
{
public:
  Init_MycobotPumpStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MycobotPumpStatus_pin1 status(::mycobot_interfaces::msg::MycobotPumpStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MycobotPumpStatus_pin1(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotPumpStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::msg::MycobotPumpStatus>()
{
  return mycobot_interfaces::msg::builder::Init_MycobotPumpStatus_status();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__BUILDER_HPP_
