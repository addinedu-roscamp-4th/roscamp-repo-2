// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:msg/RobodineCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/robodine_coords.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__BUILDER_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/msg/detail/robodine_coords__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobodineCoords_vel
{
public:
  explicit Init_RobodineCoords_vel(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::msg::RobodineCoords vel(::mycobot_interfaces::msg::RobodineCoords::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_gripper
{
public:
  explicit Init_RobodineCoords_gripper(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_vel gripper(::mycobot_interfaces::msg::RobodineCoords::_gripper_type arg)
  {
    msg_.gripper = std::move(arg);
    return Init_RobodineCoords_vel(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_rz
{
public:
  explicit Init_RobodineCoords_rz(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_gripper rz(::mycobot_interfaces::msg::RobodineCoords::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return Init_RobodineCoords_gripper(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_ry
{
public:
  explicit Init_RobodineCoords_ry(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_rz ry(::mycobot_interfaces::msg::RobodineCoords::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_RobodineCoords_rz(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_rx
{
public:
  explicit Init_RobodineCoords_rx(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_ry rx(::mycobot_interfaces::msg::RobodineCoords::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_RobodineCoords_ry(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_z
{
public:
  explicit Init_RobodineCoords_z(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_rx z(::mycobot_interfaces::msg::RobodineCoords::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_RobodineCoords_rx(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_y
{
public:
  explicit Init_RobodineCoords_y(::mycobot_interfaces::msg::RobodineCoords & msg)
  : msg_(msg)
  {}
  Init_RobodineCoords_z y(::mycobot_interfaces::msg::RobodineCoords::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobodineCoords_z(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

class Init_RobodineCoords_x
{
public:
  Init_RobodineCoords_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobodineCoords_y x(::mycobot_interfaces::msg::RobodineCoords::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobodineCoords_y(msg_);
  }

private:
  ::mycobot_interfaces::msg::RobodineCoords msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::msg::RobodineCoords>()
{
  return mycobot_interfaces::msg::builder::Init_RobodineCoords_x();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__BUILDER_HPP_
