// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/MycobotAngles.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/mycobot_angles__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotAngles__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3e, 0xa1, 0x58, 0x7c, 0x8f, 0x06, 0x24, 0x50,
      0xd8, 0xb7, 0x96, 0xe2, 0x38, 0x6f, 0x81, 0x9f,
      0xce, 0x89, 0xa9, 0xc3, 0xbb, 0x11, 0x1c, 0x9b,
      0x5b, 0x20, 0x3d, 0x80, 0x9b, 0x6b, 0x30, 0x67,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__MycobotAngles__TYPE_NAME[] = "mycobot_interfaces/msg/MycobotAngles";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_1[] = "joint_1";
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_2[] = "joint_2";
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_3[] = "joint_3";
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_4[] = "joint_4";
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_5[] = "joint_5";
static char mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_6[] = "joint_6";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__MycobotAngles__FIELDS[] = {
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_1, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_2, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_3, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_4, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_5, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotAngles__FIELD_NAME__joint_6, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__MycobotAngles__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__MycobotAngles__TYPE_NAME, 36, 36},
      {mycobot_interfaces__msg__MycobotAngles__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 joint_1\n"
  "float32 joint_2\n"
  "float32 joint_3\n"
  "float32 joint_4\n"
  "float32 joint_5\n"
  "float32 joint_6";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotAngles__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__MycobotAngles__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 96, 96},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotAngles__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__MycobotAngles__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
