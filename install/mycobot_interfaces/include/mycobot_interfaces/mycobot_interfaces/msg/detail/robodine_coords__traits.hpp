// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mycobot_interfaces:msg/RobodineCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/robodine_coords.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__TRAITS_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mycobot_interfaces/msg/detail/robodine_coords__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mycobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobodineCoords & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: rx
  {
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << ", ";
  }

  // member: ry
  {
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
    out << ", ";
  }

  // member: rz
  {
    out << "rz: ";
    rosidl_generator_traits::value_to_yaml(msg.rz, out);
    out << ", ";
  }

  // member: gripper
  {
    out << "gripper: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper, out);
    out << ", ";
  }

  // member: vel
  {
    out << "vel: ";
    rosidl_generator_traits::value_to_yaml(msg.vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobodineCoords & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rx: ";
    rosidl_generator_traits::value_to_yaml(msg.rx, out);
    out << "\n";
  }

  // member: ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ry: ";
    rosidl_generator_traits::value_to_yaml(msg.ry, out);
    out << "\n";
  }

  // member: rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rz: ";
    rosidl_generator_traits::value_to_yaml(msg.rz, out);
    out << "\n";
  }

  // member: gripper
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper, out);
    out << "\n";
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel: ";
    rosidl_generator_traits::value_to_yaml(msg.vel, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobodineCoords & msg, bool use_flow_style = false)
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
  const mycobot_interfaces::msg::RobodineCoords & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::msg::RobodineCoords & msg)
{
  return mycobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::msg::RobodineCoords>()
{
  return "mycobot_interfaces::msg::RobodineCoords";
}

template<>
inline const char * name<mycobot_interfaces::msg::RobodineCoords>()
{
  return "mycobot_interfaces/msg/RobodineCoords";
}

template<>
struct has_fixed_size<mycobot_interfaces::msg::RobodineCoords>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mycobot_interfaces::msg::RobodineCoords>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mycobot_interfaces::msg::RobodineCoords>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__ROBODINE_COORDS__TRAITS_HPP_
