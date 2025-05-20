// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:srv/GetCoords.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/srv/detail/get_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GetCoords__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe6, 0xab, 0x29, 0xcd, 0xed, 0x84, 0xa3, 0xd6,
      0x33, 0x14, 0x9d, 0x60, 0x4a, 0x82, 0x59, 0x61,
      0x85, 0xa7, 0x77, 0x82, 0xc6, 0x36, 0x53, 0xc6,
      0x74, 0x18, 0x5d, 0xaa, 0x58, 0xd7, 0xe1, 0x1e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GetCoords_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xad, 0xe4, 0x4e, 0x45, 0x8c, 0x13, 0xdf, 0xe1,
      0xac, 0x21, 0xf5, 0xb9, 0x17, 0x83, 0x31, 0xff,
      0xbd, 0xaf, 0x09, 0x19, 0x5f, 0x37, 0x5e, 0x5a,
      0xa6, 0xdf, 0x6b, 0x4a, 0xca, 0x98, 0xd5, 0xa4,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GetCoords_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5f, 0x45, 0x9c, 0x42, 0x2a, 0xdc, 0x71, 0x8d,
      0x8f, 0x5c, 0x61, 0x25, 0x83, 0xc2, 0x2b, 0xcb,
      0x09, 0x9b, 0x95, 0x5c, 0x6e, 0x08, 0x16, 0x6e,
      0x47, 0x70, 0x16, 0x45, 0x57, 0xd2, 0x52, 0xb7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GetCoords_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x9f, 0x02, 0x56, 0x89, 0x42, 0xe3, 0xf2,
      0x76, 0x86, 0xa5, 0x49, 0x03, 0xa9, 0xab, 0x3d,
      0xa6, 0x2c, 0xa6, 0xdd, 0x5e, 0x29, 0xfb, 0x35,
      0xb7, 0x6b, 0x8b, 0x03, 0x0d, 0x73, 0x52, 0x53,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char mycobot_interfaces__srv__GetCoords__TYPE_NAME[] = "mycobot_interfaces/srv/GetCoords";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mycobot_interfaces__srv__GetCoords_Event__TYPE_NAME[] = "mycobot_interfaces/srv/GetCoords_Event";
static char mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME[] = "mycobot_interfaces/srv/GetCoords_Request";
static char mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME[] = "mycobot_interfaces/srv/GetCoords_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mycobot_interfaces__srv__GetCoords__FIELD_NAME__request_message[] = "request_message";
static char mycobot_interfaces__srv__GetCoords__FIELD_NAME__response_message[] = "response_message";
static char mycobot_interfaces__srv__GetCoords__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GetCoords__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GetCoords__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GetCoords_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__GetCoords__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__GetCoords__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GetCoords__TYPE_NAME, 32, 32},
      {mycobot_interfaces__srv__GetCoords__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__GetCoords__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__GetCoords_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__GetCoords_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mycobot_interfaces__srv__GetCoords_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GetCoords_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GetCoords_Request__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GetCoords_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__GetCoords_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
      {mycobot_interfaces__srv__GetCoords_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__x[] = "x";
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__y[] = "y";
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__z[] = "z";
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__rx[] = "rx";
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__ry[] = "ry";
static char mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__rz[] = "rz";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GetCoords_Response__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__FIELD_NAME__rz, 2, 2},
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
mycobot_interfaces__srv__GetCoords_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
      {mycobot_interfaces__srv__GetCoords_Response__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__info[] = "info";
static char mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__request[] = "request";
static char mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GetCoords_Event__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__GetCoords_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__GetCoords_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GetCoords_Event__TYPE_NAME, 38, 38},
      {mycobot_interfaces__srv__GetCoords_Event__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__GetCoords_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__GetCoords_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__GetCoords_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "float32 rx\n"
  "float32 ry\n"
  "float32 rz";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GetCoords__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GetCoords__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 66, 66},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GetCoords_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GetCoords_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GetCoords_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GetCoords_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GetCoords_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GetCoords_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GetCoords__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GetCoords__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__GetCoords_Event__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__GetCoords_Request__get_individual_type_description_source(NULL);
    sources[4] = *mycobot_interfaces__srv__GetCoords_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GetCoords_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GetCoords_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GetCoords_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GetCoords_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GetCoords_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GetCoords_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__GetCoords_Request__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__GetCoords_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
