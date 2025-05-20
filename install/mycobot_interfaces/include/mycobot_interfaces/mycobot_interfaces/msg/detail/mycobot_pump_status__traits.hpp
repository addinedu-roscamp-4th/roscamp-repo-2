// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mycobot_interfaces:msg/MycobotPumpStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/mycobot_pump_status.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__TRAITS_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mycobot_interfaces/msg/detail/mycobot_pump_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mycobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MycobotPumpStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: pin1
  {
    out << "pin1: ";
    rosidl_generator_traits::value_to_yaml(msg.pin1, out);
    out << ", ";
  }

  // member: pin2
  {
    out << "pin2: ";
    rosidl_generator_traits::value_to_yaml(msg.pin2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MycobotPumpStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: pin1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pin1: ";
    rosidl_generator_traits::value_to_yaml(msg.pin1, out);
    out << "\n";
  }

  // member: pin2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pin2: ";
    rosidl_generator_traits::value_to_yaml(msg.pin2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MycobotPumpStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mycobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mycobot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mycobot_interfaces::msg::MycobotPumpStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::msg::MycobotPumpStatus & msg)
{
  return mycobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::msg::MycobotPumpStatus>()
{
  return "mycobot_interfaces::msg::MycobotPumpStatus";
}

template<>
inline const char * name<mycobot_interfaces::msg::MycobotPumpStatus>()
{
  return "mycobot_interfaces/msg/MycobotPumpStatus";
}

template<>
struct has_fixed_size<mycobot_interfaces::msg::MycobotPumpStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mycobot_interfaces::msg::MycobotPumpStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mycobot_interfaces::msg::MycobotPumpStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__TRAITS_HPP_
