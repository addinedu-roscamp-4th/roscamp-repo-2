// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mycobot_interfaces:msg/MycobotAngles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_angles.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__BUILDER_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mycobot_interfaces/msg/detail/mycobot_angles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mycobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MycobotAngles_joint_6
{
public:
  explicit Init_MycobotAngles_joint_6(::mycobot_interfaces::msg::MycobotAngles & msg)
  : msg_(msg)
  {}
  ::mycobot_interfaces::msg::MycobotAngles joint_6(::mycobot_interfaces::msg::MycobotAngles::_joint_6_type arg)
  {
    msg_.joint_6 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

class Init_MycobotAngles_joint_5
{
public:
  explicit Init_MycobotAngles_joint_5(::mycobot_interfaces::msg::MycobotAngles & msg)
  : msg_(msg)
  {}
  Init_MycobotAngles_joint_6 joint_5(::mycobot_interfaces::msg::MycobotAngles::_joint_5_type arg)
  {
    msg_.joint_5 = std::move(arg);
    return Init_MycobotAngles_joint_6(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

class Init_MycobotAngles_joint_4
{
public:
  explicit Init_MycobotAngles_joint_4(::mycobot_interfaces::msg::MycobotAngles & msg)
  : msg_(msg)
  {}
  Init_MycobotAngles_joint_5 joint_4(::mycobot_interfaces::msg::MycobotAngles::_joint_4_type arg)
  {
    msg_.joint_4 = std::move(arg);
    return Init_MycobotAngles_joint_5(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

class Init_MycobotAngles_joint_3
{
public:
  explicit Init_MycobotAngles_joint_3(::mycobot_interfaces::msg::MycobotAngles & msg)
  : msg_(msg)
  {}
  Init_MycobotAngles_joint_4 joint_3(::mycobot_interfaces::msg::MycobotAngles::_joint_3_type arg)
  {
    msg_.joint_3 = std::move(arg);
    return Init_MycobotAngles_joint_4(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

class Init_MycobotAngles_joint_2
{
public:
  explicit Init_MycobotAngles_joint_2(::mycobot_interfaces::msg::MycobotAngles & msg)
  : msg_(msg)
  {}
  Init_MycobotAngles_joint_3 joint_2(::mycobot_interfaces::msg::MycobotAngles::_joint_2_type arg)
  {
    msg_.joint_2 = std::move(arg);
    return Init_MycobotAngles_joint_3(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

class Init_MycobotAngles_joint_1
{
public:
  Init_MycobotAngles_joint_1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MycobotAngles_joint_2 joint_1(::mycobot_interfaces::msg::MycobotAngles::_joint_1_type arg)
  {
    msg_.joint_1 = std::move(arg);
    return Init_MycobotAngles_joint_2(msg_);
  }

private:
  ::mycobot_interfaces::msg::MycobotAngles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mycobot_interfaces::msg::MycobotAngles>()
{
  return mycobot_interfaces::msg::builder::Init_MycobotAngles_joint_1();
}

}  // namespace mycobot_interfaces

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_ANGLES__BUILDER_HPP_
