// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mycobot_interfaces:srv/CookGPTsrv.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mycobot_interfaces/srv/cook_gp_tsrv.h"


#ifndef MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__STRUCT_H_
#define MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CookGPTsrv in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Request
{
  int8_t command;
  rosidl_runtime_c__String robot_id;
} mycobot_interfaces__srv__CookGPTsrv_Request;

// Struct for a sequence of mycobot_interfaces__srv__CookGPTsrv_Request.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Request__Sequence
{
  mycobot_interfaces__srv__CookGPTsrv_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__CookGPTsrv_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/CookGPTsrv in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Response
{
  float x;
  float y;
  float z;
  float rx;
  float ry;
  float rz;
  bool dish;
  bool sauce;
  bool stain;
} mycobot_interfaces__srv__CookGPTsrv_Response;

// Struct for a sequence of mycobot_interfaces__srv__CookGPTsrv_Response.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Response__Sequence
{
  mycobot_interfaces__srv__CookGPTsrv_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__CookGPTsrv_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mycobot_interfaces__srv__CookGPTsrv_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mycobot_interfaces__srv__CookGPTsrv_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CookGPTsrv in the package mycobot_interfaces.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mycobot_interfaces__srv__CookGPTsrv_Request__Sequence request;
  mycobot_interfaces__srv__CookGPTsrv_Response__Sequence response;
} mycobot_interfaces__srv__CookGPTsrv_Event;

// Struct for a sequence of mycobot_interfaces__srv__CookGPTsrv_Event.
typedef struct mycobot_interfaces__srv__CookGPTsrv_Event__Sequence
{
  mycobot_interfaces__srv__CookGPTsrv_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mycobot_interfaces__srv__CookGPTsrv_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYCOBOT_INTERFACES__SRV__DETAIL__COOK_GP_TSRV__STRUCT_H_
