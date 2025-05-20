// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:msg/RobodineCoords.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/msg/detail/robodine_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__msg__RobodineCoords__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8c, 0x20, 0x68, 0x6b, 0x0d, 0xb5, 0x2c, 0xc0,
      0xf7, 0x6a, 0x48, 0x13, 0xea, 0xc8, 0x34, 0x58,
      0x51, 0xeb, 0x5d, 0x2b, 0x1e, 0xab, 0xaa, 0x17,
      0xd3, 0x5e, 0xa4, 0xd6, 0x13, 0x67, 0x29, 0x7b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mycobot_interfaces__msg__RobodineCoords__TYPE_NAME[] = "mycobot_interfaces/msg/RobodineCoords";

// Define type names, field names, and default values
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__x[] = "x";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__y[] = "y";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__z[] = "z";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__rx[] = "rx";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__ry[] = "ry";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__rz[] = "rz";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__gripper[] = "gripper";
static char mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__vel[] = "vel";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__msg__RobodineCoords__FIELDS[] = {
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__gripper, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__msg__RobodineCoords__FIELD_NAME__vel, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__msg__RobodineCoords__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__msg__RobodineCoords__TYPE_NAME, 37, 37},
      {mycobot_interfaces__msg__RobodineCoords__FIELDS, 8, 8},
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
  "float32 rz\n"
  "int32 gripper\n"
  "int32 vel";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__msg__RobodineCoords__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__msg__RobodineCoords__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 86, 86},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__msg__RobodineCoords__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__msg__RobodineCoords__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
