// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from serial_motor_demo_msgs:msg/MotorVels.idl
// generated code does not contain a copyright notice
#include "serial_motor_demo_msgs/msg/detail/motor_vels__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "serial_motor_demo_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "serial_motor_demo_msgs/msg/detail/motor_vels__struct.h"
#include "serial_motor_demo_msgs/msg/detail/motor_vels__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MotorVels__ros_msg_type = serial_motor_demo_msgs__msg__MotorVels;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
bool cdr_serialize_serial_motor_demo_msgs__msg__MotorVels(
  const serial_motor_demo_msgs__msg__MotorVels * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: mot_1_rad_sec
  {
    cdr << ros_message->mot_1_rad_sec;
  }

  // Field name: mot_2_rad_sec
  {
    cdr << ros_message->mot_2_rad_sec;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
bool cdr_deserialize_serial_motor_demo_msgs__msg__MotorVels(
  eprosima::fastcdr::Cdr & cdr,
  serial_motor_demo_msgs__msg__MotorVels * ros_message)
{
  // Field name: mot_1_rad_sec
  {
    cdr >> ros_message->mot_1_rad_sec;
  }

  // Field name: mot_2_rad_sec
  {
    cdr >> ros_message->mot_2_rad_sec;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
size_t get_serialized_size_serial_motor_demo_msgs__msg__MotorVels(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorVels__ros_msg_type * ros_message = static_cast<const _MotorVels__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: mot_1_rad_sec
  {
    size_t item_size = sizeof(ros_message->mot_1_rad_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: mot_2_rad_sec
  {
    size_t item_size = sizeof(ros_message->mot_2_rad_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
size_t max_serialized_size_serial_motor_demo_msgs__msg__MotorVels(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: mot_1_rad_sec
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: mot_2_rad_sec
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = serial_motor_demo_msgs__msg__MotorVels;
    is_plain =
      (
      offsetof(DataType, mot_2_rad_sec) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
bool cdr_serialize_key_serial_motor_demo_msgs__msg__MotorVels(
  const serial_motor_demo_msgs__msg__MotorVels * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: mot_1_rad_sec
  {
    cdr << ros_message->mot_1_rad_sec;
  }

  // Field name: mot_2_rad_sec
  {
    cdr << ros_message->mot_2_rad_sec;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
size_t get_serialized_size_key_serial_motor_demo_msgs__msg__MotorVels(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorVels__ros_msg_type * ros_message = static_cast<const _MotorVels__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: mot_1_rad_sec
  {
    size_t item_size = sizeof(ros_message->mot_1_rad_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: mot_2_rad_sec
  {
    size_t item_size = sizeof(ros_message->mot_2_rad_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_serial_motor_demo_msgs
size_t max_serialized_size_key_serial_motor_demo_msgs__msg__MotorVels(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: mot_1_rad_sec
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: mot_2_rad_sec
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = serial_motor_demo_msgs__msg__MotorVels;
    is_plain =
      (
      offsetof(DataType, mot_2_rad_sec) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _MotorVels__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const serial_motor_demo_msgs__msg__MotorVels * ros_message = static_cast<const serial_motor_demo_msgs__msg__MotorVels *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_serial_motor_demo_msgs__msg__MotorVels(ros_message, cdr);
}

static bool _MotorVels__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  serial_motor_demo_msgs__msg__MotorVels * ros_message = static_cast<serial_motor_demo_msgs__msg__MotorVels *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_serial_motor_demo_msgs__msg__MotorVels(cdr, ros_message);
}

static uint32_t _MotorVels__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_serial_motor_demo_msgs__msg__MotorVels(
      untyped_ros_message, 0));
}

static size_t _MotorVels__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_serial_motor_demo_msgs__msg__MotorVels(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotorVels = {
  "serial_motor_demo_msgs::msg",
  "MotorVels",
  _MotorVels__cdr_serialize,
  _MotorVels__cdr_deserialize,
  _MotorVels__get_serialized_size,
  _MotorVels__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MotorVels__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorVels,
  get_message_typesupport_handle_function,
  &serial_motor_demo_msgs__msg__MotorVels__get_type_hash,
  &serial_motor_demo_msgs__msg__MotorVels__get_type_description,
  &serial_motor_demo_msgs__msg__MotorVels__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, serial_motor_demo_msgs, msg, MotorVels)() {
  return &_MotorVels__type_support;
}

#if defined(__cplusplus)
}
#endif
