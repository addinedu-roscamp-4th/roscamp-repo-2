// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/MycobotPumpStatus.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/mycobot_pump_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotPumpStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0x80, 0xb7, 0xca, 0x23, 0x39, 0xab, 0x5b,
      0x16, 0xe6, 0x76, 0x1f, 0xda, 0xda, 0xd7, 0x0c,
      0x3c, 0x3e, 0xf1, 0xce, 0x30, 0x2b, 0xe8, 0x5d,
      0xe8, 0xe8, 0xec, 0x23, 0x16, 0xa6, 0x26, 0x2b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__MycobotPumpStatus__TYPE_NAME[] = "mycobot_interfaces/msg/MycobotPumpStatus";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__status[] = "status";
static char mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__pin1[] = "pin1";
static char mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__pin2[] = "pin2";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__MycobotPumpStatus__FIELDS[] = {
  {
    {mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__pin1, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotPumpStatus__FIELD_NAME__pin2, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__MycobotPumpStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__MycobotPumpStatus__TYPE_NAME, 40, 40},
      {mycobot_interfaces__msg__MycobotPumpStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool status\n"
  "int8 pin1\n"
  "int8 pin2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotPumpStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__MycobotPumpStatus__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 32, 32},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotPumpStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__MycobotPumpStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
