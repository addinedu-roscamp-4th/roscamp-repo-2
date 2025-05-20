// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robodine_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robodine_msgs/msg/robot_status.hpp"


#ifndef ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robodine_msgs/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robodine_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_timestamp
{
public:
  explicit Init_RobotStatus_timestamp(::robodine_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::robodine_msgs::msg::RobotStatus timestamp(::robodine_msgs::msg::RobotStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_battery_level
{
public:
  explicit Init_RobotStatus_battery_level(::robodine_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_timestamp battery_level(::robodine_msgs::msg::RobotStatus::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_RobotStatus_timestamp(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_position
{
public:
  explicit Init_RobotStatus_position(::robodine_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_battery_level position(::robodine_msgs::msg::RobotStatus::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_RobotStatus_battery_level(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_status
{
public:
  explicit Init_RobotStatus_status(::robodine_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_position status(::robodine_msgs::msg::RobotStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotStatus_position(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_type
{
public:
  explicit Init_RobotStatus_type(::robodine_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_status type(::robodine_msgs::msg::RobotStatus::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_RobotStatus_status(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_id
{
public:
  Init_RobotStatus_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_type id(::robodine_msgs::msg::RobotStatus::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RobotStatus_type(msg_);
  }

private:
  ::robodine_msgs::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robodine_msgs::msg::RobotStatus>()
{
  return robodine_msgs::msg::builder::Init_RobotStatus_id();
}

}  // namespace robodine_msgs

#endif  // ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
