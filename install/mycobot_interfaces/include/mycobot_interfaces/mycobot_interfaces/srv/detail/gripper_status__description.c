// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mycobot_interfaces:srv/GripperStatus.idl
// generated code does not contain a copyright notice

#include "mycobot_interfaces/srv/detail/gripper_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GripperStatus__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x05, 0x54, 0x15, 0x58, 0x18, 0xab, 0xa4, 0x77,
      0xec, 0x37, 0x1a, 0xec, 0x2a, 0xf7, 0x06, 0x91,
      0x84, 0x76, 0x93, 0x63, 0xe6, 0xdf, 0x39, 0x7a,
      0xae, 0x53, 0x5e, 0x80, 0xf0, 0x15, 0x6a, 0x67,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GripperStatus_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x95, 0xbd, 0x5b, 0xec, 0x02, 0xf4, 0x72, 0x80,
      0x45, 0x6f, 0xa6, 0xa4, 0x91, 0xc9, 0x90, 0x69,
      0xcc, 0x16, 0xe6, 0x9b, 0xf7, 0x1d, 0xd9, 0x62,
      0x83, 0x6e, 0x71, 0xa9, 0x2b, 0xdd, 0x97, 0xdd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GripperStatus_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x61, 0xae, 0xc1, 0x45, 0x36, 0xae, 0x87, 0x51,
      0xe1, 0xbd, 0x3e, 0x38, 0x81, 0xc4, 0xdf, 0x68,
      0x8e, 0x6a, 0x14, 0xfd, 0x77, 0x7c, 0xd8, 0xfe,
      0xe2, 0x29, 0xfa, 0x22, 0x0b, 0x20, 0x3b, 0x96,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mycobot_interfaces
const rosidl_type_hash_t *
mycobot_interfaces__srv__GripperStatus_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0x83, 0x56, 0x0a, 0x21, 0xcf, 0xb2, 0x7e,
      0x1c, 0x64, 0x9d, 0x49, 0x1b, 0x03, 0x25, 0x59,
      0x1a, 0x4b, 0xa6, 0x10, 0xeb, 0x54, 0xd0, 0x65,
      0xd6, 0x19, 0xc4, 0x7a, 0x6c, 0x46, 0xf6, 0x11,
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

static char mycobot_interfaces__srv__GripperStatus__TYPE_NAME[] = "mycobot_interfaces/srv/GripperStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mycobot_interfaces__srv__GripperStatus_Event__TYPE_NAME[] = "mycobot_interfaces/srv/GripperStatus_Event";
static char mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME[] = "mycobot_interfaces/srv/GripperStatus_Request";
static char mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME[] = "mycobot_interfaces/srv/GripperStatus_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mycobot_interfaces__srv__GripperStatus__FIELD_NAME__request_message[] = "request_message";
static char mycobot_interfaces__srv__GripperStatus__FIELD_NAME__response_message[] = "response_message";
static char mycobot_interfaces__srv__GripperStatus__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GripperStatus__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GripperStatus__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mycobot_interfaces__srv__GripperStatus_Event__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__GripperStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Event__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__GripperStatus__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GripperStatus__TYPE_NAME, 36, 36},
      {mycobot_interfaces__srv__GripperStatus__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__GripperStatus__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__GripperStatus_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__GripperStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mycobot_interfaces__srv__GripperStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GripperStatus_Request__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GripperStatus_Request__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GripperStatus_Request__FIELD_NAME__status, 6, 6},
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
mycobot_interfaces__srv__GripperStatus_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
      {mycobot_interfaces__srv__GripperStatus_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GripperStatus_Response__FIELD_NAME__flag[] = "flag";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GripperStatus_Response__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GripperStatus_Response__FIELD_NAME__flag, 4, 4},
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
mycobot_interfaces__srv__GripperStatus_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
      {mycobot_interfaces__srv__GripperStatus_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__info[] = "info";
static char mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__request[] = "request";
static char mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mycobot_interfaces__srv__GripperStatus_Event__FIELDS[] = {
  {
    {mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mycobot_interfaces__srv__GripperStatus_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mycobot_interfaces__srv__GripperStatus_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mycobot_interfaces__srv__GripperStatus_Event__TYPE_NAME, 42, 42},
      {mycobot_interfaces__srv__GripperStatus_Event__FIELDS, 3, 3},
    },
    {mycobot_interfaces__srv__GripperStatus_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mycobot_interfaces__srv__GripperStatus_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mycobot_interfaces__srv__GripperStatus_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool status\n"
  "---\n"
  "bool flag";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GripperStatus__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GripperStatus__TYPE_NAME, 36, 36},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 25, 25},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GripperStatus_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GripperStatus_Request__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GripperStatus_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GripperStatus_Response__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mycobot_interfaces__srv__GripperStatus_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mycobot_interfaces__srv__GripperStatus_Event__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GripperStatus__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GripperStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__GripperStatus_Event__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__GripperStatus_Request__get_individual_type_description_source(NULL);
    sources[4] = *mycobot_interfaces__srv__GripperStatus_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GripperStatus_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GripperStatus_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GripperStatus_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GripperStatus_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mycobot_interfaces__srv__GripperStatus_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mycobot_interfaces__srv__GripperStatus_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mycobot_interfaces__srv__GripperStatus_Request__get_individual_type_description_source(NULL);
    sources[3] = *mycobot_interfaces__srv__GripperStatus_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
