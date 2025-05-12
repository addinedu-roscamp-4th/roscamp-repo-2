// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/MycobotCoords.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/mycobot_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__MycobotCoords__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x46, 0x39, 0xb3, 0x32, 0x37, 0x32, 0xf9, 0xa3,
      0x5c, 0x25, 0x52, 0x8a, 0xdd, 0x03, 0xd9, 0xc3,
      0xe7, 0xe6, 0xfb, 0x55, 0x53, 0x51, 0xa1, 0x99,
      0x69, 0x24, 0xf8, 0x8d, 0x6d, 0xc0, 0x79, 0xb4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__MycobotCoords__TYPE_NAME[] = "mycobot_interfaces/msg/MycobotCoords";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__x[] = "x";
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__y[] = "y";
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__z[] = "z";
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__rx[] = "rx";
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__ry[] = "ry";
static char mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__rz[] = "rz";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__MycobotCoords__FIELDS[] = {
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__MycobotCoords__FIELD_NAME__rz, 2, 2},
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
mycobot_interfaces__msg__MycobotCoords__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__MycobotCoords__TYPE_NAME, 36, 36},
      {mycobot_interfaces__msg__MycobotCoords__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 rx\n"
  "float32 ry\n"
  "float32 rz";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__MycobotCoords__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__MycobotCoords__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 62, 62},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__MycobotCoords__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__MycobotCoords__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
