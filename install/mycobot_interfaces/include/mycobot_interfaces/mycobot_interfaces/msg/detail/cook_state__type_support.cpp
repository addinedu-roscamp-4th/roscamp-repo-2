// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mycobot_interfaces:msg/CookState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mycobot_interfaces/msg/detail/cook_state__functions.h"
#include "mycobot_interfaces/msg/detail/cook_state__struct.hpp"
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

void CookState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mycobot_interfaces::msg::CookState(_init);
}

void CookState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mycobot_interfaces::msg::CookState *>(message_memory);
  typed_message->~CookState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CookState_message_member_array[2] = {
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces::msg::CookState, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "order_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces::msg::CookState, order_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CookState_message_members = {
  "mycobot_interfaces::msg",  // message namespace
  "CookState",  // message name
  2,  // number of fields
  sizeof(mycobot_interfaces::msg::CookState),
  false,  // has_any_key_member_
  CookState_message_member_array,  // message members
  CookState_init_function,  // function to initialize message memory (memory has to be allocated)
  CookState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CookState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CookState_message_members,
  get_message_typesupport_handle_function,
  &mycobot_interfaces__msg__CookState__get_type_hash,
  &mycobot_interfaces__msg__CookState__get_type_description,
  &mycobot_interfaces__msg__CookState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mycobot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mycobot_interfaces::msg::CookState>()
{
  return &::mycobot_interfaces::msg::rosidl_typesupport_introspection_cpp::CookState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mycobot_interfaces, msg, CookState)() {
  return &::mycobot_interfaces::msg::rosidl_typesupport_introspection_cpp::CookState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
