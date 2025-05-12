// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mycobot_interfaces:msg/MycobotGripperStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mycobot_interfaces/msg/detail/mycobot_gripper_status__functions.h"
#include "mycobot_interfaces/msg/detail/mycobot_gripper_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mycobot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MycobotGripperStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mycobot_interfaces::msg::MycobotGripperStatus(_init);
}

void MycobotGripperStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mycobot_interfaces::msg::MycobotGripperStatus *>(message_memory);
  typed_message->~MycobotGripperStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MycobotGripperStatus_message_member_array[1] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces::msg::MycobotGripperStatus, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MycobotGripperStatus_message_members = {
  "mycobot_interfaces::msg",  // message namespace
  "MycobotGripperStatus",  // message name
  1,  // number of fields
  sizeof(mycobot_interfaces::msg::MycobotGripperStatus),
  false,  // has_any_key_member_
  MycobotGripperStatus_message_member_array,  // message members
  MycobotGripperStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MycobotGripperStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MycobotGripperStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MycobotGripperStatus_message_members,
  get_message_typesupport_handle_function,
  &mycobot_interfaces__msg__MycobotGripperStatus__get_type_hash,
  &mycobot_interfaces__msg__MycobotGripperStatus__get_type_description,
  &mycobot_interfaces__msg__MycobotGripperStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mycobot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mycobot_interfaces::msg::MycobotGripperStatus>()
{
  return &::mycobot_interfaces::msg::rosidl_typesupport_introspection_cpp::MycobotGripperStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mycobot_interfaces, msg, MycobotGripperStatus)() {
  return &::mycobot_interfaces::msg::rosidl_typesupport_introspection_cpp::MycobotGripperStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
