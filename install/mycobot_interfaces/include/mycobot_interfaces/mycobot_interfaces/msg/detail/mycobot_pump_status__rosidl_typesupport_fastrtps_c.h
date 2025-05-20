// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from mycobot_interfaces:msg/MycobotPumpStatus.idl
// generated code does not contain a copyright notice
#ifndef MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mycobot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mycobot_interfaces/msg/detail/mycobot_pump_status__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
bool cdr_serialize_mycobot_interfaces__msg__MycobotPumpStatus(
  const mycobot_interfaces__msg__MycobotPumpStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
bool cdr_deserialize_mycobot_interfaces__msg__MycobotPumpStatus(
  eprosima::fastcdr::Cdr &,
  mycobot_interfaces__msg__MycobotPumpStatus * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
size_t get_serialized_size_mycobot_interfaces__msg__MycobotPumpStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
size_t max_serialized_size_mycobot_interfaces__msg__MycobotPumpStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
bool cdr_serialize_key_mycobot_interfaces__msg__MycobotPumpStatus(
  const mycobot_interfaces__msg__MycobotPumpStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
size_t get_serialized_size_key_mycobot_interfaces__msg__MycobotPumpStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
size_t max_serialized_size_key_mycobot_interfaces__msg__MycobotPumpStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mycobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mycobot_interfaces, msg, MycobotPumpStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__MSG__DETAIL__MYCOBOT_PUMP_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
