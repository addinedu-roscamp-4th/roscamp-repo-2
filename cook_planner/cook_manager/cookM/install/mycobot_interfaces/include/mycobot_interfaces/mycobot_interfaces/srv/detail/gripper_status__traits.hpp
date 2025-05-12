// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mycobot_interfaces:srv/GripperStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/gripper_status.hpp"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__TRAITS_HPP_
#define MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mycobot_interfaces/srv/detail/gripper_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mycobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperStatus_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperStatus_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperStatus_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mycobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mycobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mycobot_interfaces::srv::GripperStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::srv::GripperStatus_Request & msg)
{
  return mycobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::srv::GripperStatus_Request>()
{
  return "mycobot_interfaces::srv::GripperStatus_Request";
}

template<>
inline const char * name<mycobot_interfaces::srv::GripperStatus_Request>()
{
  return "mycobot_interfaces/srv/GripperStatus_Request";
}

template<>
struct has_fixed_size<mycobot_interfaces::srv::GripperStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mycobot_interfaces::srv::GripperStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mycobot_interfaces::srv::GripperStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mycobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperStatus_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mycobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mycobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mycobot_interfaces::srv::GripperStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::srv::GripperStatus_Response & msg)
{
  return mycobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::srv::GripperStatus_Response>()
{
  return "mycobot_interfaces::srv::GripperStatus_Response";
}

template<>
inline const char * name<mycobot_interfaces::srv::GripperStatus_Response>()
{
  return "mycobot_interfaces/srv/GripperStatus_Response";
}

template<>
struct has_fixed_size<mycobot_interfaces::srv::GripperStatus_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mycobot_interfaces::srv::GripperStatus_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mycobot_interfaces::srv::GripperStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mycobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperStatus_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperStatus_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mycobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mycobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mycobot_interfaces::srv::GripperStatus_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mycobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mycobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const mycobot_interfaces::srv::GripperStatus_Event & msg)
{
  return mycobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mycobot_interfaces::srv::GripperStatus_Event>()
{
  return "mycobot_interfaces::srv::GripperStatus_Event";
}

template<>
inline const char * name<mycobot_interfaces::srv::GripperStatus_Event>()
{
  return "mycobot_interfaces/srv/GripperStatus_Event";
}

template<>
struct has_fixed_size<mycobot_interfaces::srv::GripperStatus_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mycobot_interfaces::srv::GripperStatus_Event>
  : std::integral_constant<bool, has_bounded_size<mycobot_interfaces::srv::GripperStatus_Request>::value && has_bounded_size<mycobot_interfaces::srv::GripperStatus_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mycobot_interfaces::srv::GripperStatus_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mycobot_interfaces::srv::GripperStatus>()
{
  return "mycobot_interfaces::srv::GripperStatus";
}

template<>
inline const char * name<mycobot_interfaces::srv::GripperStatus>()
{
  return "mycobot_interfaces/srv/GripperStatus";
}

template<>
struct has_fixed_size<mycobot_interfaces::srv::GripperStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<mycobot_interfaces::srv::GripperStatus_Request>::value &&
    has_fixed_size<mycobot_interfaces::srv::GripperStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<mycobot_interfaces::srv::GripperStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<mycobot_interfaces::srv::GripperStatus_Request>::value &&
    has_bounded_size<mycobot_interfaces::srv::GripperStatus_Response>::value
  >
{
};

template<>
struct is_service<mycobot_interfaces::srv::GripperStatus>
  : std::true_type
{
};

template<>
struct is_service_request<mycobot_interfaces::srv::GripperStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mycobot_interfaces::srv::GripperStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__TRAITS_HPP_
