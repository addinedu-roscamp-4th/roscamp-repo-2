// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mycobot_interfaces:msg/MycobotAngles.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mycobot_interfaces/msg/detail/mycobot_angles__rosidl_typesupport_introspection_c.h"
#include "mycobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mycobot_interfaces/msg/detail/mycobot_angles__functions.h"
#include "mycobot_interfaces/msg/detail/mycobot_angles__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mycobot_interfaces__msg__MycobotAngles__init(message_memory);
}

void mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_fini_function(void * message_memory)
{
  mycobot_interfaces__msg__MycobotAngles__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_member_array[6] = {
  {
    "joint_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_6",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mycobot_interfaces__msg__MycobotAngles, joint_6),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_members = {
  "mycobot_interfaces__msg",  // message namespace
  "MycobotAngles",  // message name
  6,  // number of fields
  sizeof(mycobot_interfaces__msg__MycobotAngles),
  false,  // has_any_key_member_
  mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_member_array,  // message members
  mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_init_function,  // function to initialize message memory (memory has to be allocated)
  mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_type_support_handle = {
  0,
  &mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_members,
  get_message_typesupport_handle_function,
  &mycobot_interfaces__msg__MycobotAngles__get_type_hash,
  &mycobot_interfaces__msg__MycobotAngles__get_type_description,
  &mycobot_interfaces__msg__MycobotAngles__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mycobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mycobot_interfaces, msg, MycobotAngles)() {
  if (!mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_type_support_handle.typesupport_identifier) {
    mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mycobot_interfaces__msg__MycobotAngles__rosidl_typesupport_introspection_c__MycobotAngles_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
