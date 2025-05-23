// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/msg/cook_state.hpp"


#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__TRAITS_HPP_
#define MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mycobot_interfaces/msg/detail/cook_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mycobot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CookState & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: order_id
  {
    out << "order_id: ";
    rosidl_generator_traits::value_to_yaml(msg.order_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CookState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: order_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "order_id: ";
    rosidl_generator_traits::value_to_yaml(msg.order_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CookState & msg, bool use_flow_style = false)
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
  const mycobot_interfaces::msg::CookState & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::msg::CookState & msg)
{
  return mycobot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::msg::CookState>()
{
  return "mycobot_interfaces::msg::CookState";
}

template<>
inline const char * name<mycobot_interfaces::msg::CookState>()
{
  return "mycobot_interfaces/msg/CookState";
}

template<>
struct has_fixed_size<mycobot_interfaces::msg::CookState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mycobot_interfaces::msg::CookState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mycobot_interfaces::msg::CookState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__COOK_STATE__TRAITS_HPP_
