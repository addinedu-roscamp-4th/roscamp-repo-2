// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robodine_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robodine_msgs/msg/robot_status.h"


#ifndef ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'type'
// Member 'status'
#include "rosidl_runtime_c/string.h"
// Member 'position'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/RobotStatus in the package robodine_msgs.
/**
  * RobotStatus.msg
 */
typedef struct robodine_msgs__msg__RobotStatus
{
  int32_t id;
  rosidl_runtime_c__String type;
  rosidl_runtime_c__String status;
  geometry_msgs__msg__Pose position;
  float battery_level;
  builtin_interfaces__msg__Time timestamp;
} robodine_msgs__msg__RobotStatus;

// Struct for a sequence of robodine_msgs__msg__RobotStatus.
typedef struct robodine_msgs__msg__RobotStatus__Sequence
{
  robodine_msgs__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robodine_msgs__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBODINE_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
