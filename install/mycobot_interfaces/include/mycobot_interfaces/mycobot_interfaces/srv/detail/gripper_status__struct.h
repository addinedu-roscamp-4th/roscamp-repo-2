// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:srv/GripperStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/gripper_status.h"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__STRUCT_H_
#define MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GripperStatus in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__GripperStatus_Request
{
  bool status;
} mycobot_interfaces__srv__GripperStatus_Request;

// Struct for a sequence of mycobot_interfaces__srv__GripperStatus_Request.
typedef struct mycobot_interfaces__srv__GripperStatus_Request__Sequence
{
  mycobot_interfaces__srv__GripperStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__GripperStatus_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/GripperStatus in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__GripperStatus_Response
{
  bool flag;
} mycobot_interfaces__srv__GripperStatus_Response;

// Struct for a sequence of mycobot_interfaces__srv__GripperStatus_Response.
typedef struct mycobot_interfaces__srv__GripperStatus_Response__Sequence
{
  mycobot_interfaces__srv__GripperStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__GripperStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mycobot_interfaces__srv__GripperStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mycobot_interfaces__srv__GripperStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GripperStatus in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__GripperStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mycobot_interfaces__srv__GripperStatus_Request__Sequence request;
  mycobot_interfaces__srv__GripperStatus_Response__Sequence response;
} mycobot_interfaces__srv__GripperStatus_Event;

// Struct for a sequence of mycobot_interfaces__srv__GripperStatus_Event.
typedef struct mycobot_interfaces__srv__GripperStatus_Event__Sequence
{
  mycobot_interfaces__srv__GripperStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__GripperStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__GRIPPER_STATUS__STRUCT_H_
