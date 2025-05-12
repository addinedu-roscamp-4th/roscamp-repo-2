// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/MycobotGripperStatus.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/mycobot_gripper_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotGripperStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x94, 0xb1, 0x43, 0x1b, 0x9f, 0x6b, 0x1b, 0x26,
      0x29, 0x8b, 0xb3, 0x4c, 0xac, 0xa7, 0x09, 0x4e,
      0xc2, 0xaf, 0xd6, 0xc5, 0x83, 0x48, 0x45, 0xdb,
      0x9a, 0xe7, 0xba, 0x80, 0x83, 0xc8, 0x9f, 0xbb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__MycobotGripperStatus__TYPE_NAME[] = "mycobot_interfaces/msg/MycobotGripperStatus";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__MycobotGripperStatus__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__MycobotGripperStatus__FIELDS[] = {
  {
    {mycobot_interfaces__msg__MycobotGripperStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__MycobotGripperStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__MycobotGripperStatus__TYPE_NAME, 43, 43},
      {mycobot_interfaces__msg__MycobotGripperStatus__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool status";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotGripperStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__MycobotGripperStatus__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 11, 11},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotGripperStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__MycobotGripperStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
