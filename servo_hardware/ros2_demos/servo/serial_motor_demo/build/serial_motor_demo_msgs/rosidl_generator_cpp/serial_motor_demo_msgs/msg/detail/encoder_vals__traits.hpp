// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from serial_motor_demo_msgs:msg/EncoderVals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "serial_motor_demo_msgs/msg/encoder_vals.hpp"


#ifndef SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__ENCODER_VALS__TRAITS_HPP_
#define SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__ENCODER_VALS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "serial_motor_demo_msgs/msg/detail/encoder_vals__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace serial_motor_demo_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderVals & msg,
  std::ostream & out)
{
  out << "{";
  // member: mot_1_enc_val
  {
    out << "mot_1_enc_val: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_1_enc_val, out);
    out << ", ";
  }

  // member: mot_2_enc_val
  {
    out << "mot_2_enc_val: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_2_enc_val, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderVals & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mot_1_enc_val
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot_1_enc_val: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_1_enc_val, out);
    out << "\n";
  }

  // member: mot_2_enc_val
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mot_2_enc_val: ";
    rosidl_generator_traits::value_to_yaml(msg.mot_2_enc_val, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderVals & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace serial_motor_demo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use serial_motor_demo_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const serial_motor_demo_msgs::msg::EncoderVals & msg,
  std::ostream & out, size_t indentation = 0)
{
  serial_motor_demo_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use serial_motor_demo_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const serial_motor_demo_msgs::msg::EncoderVals & msg)
{
  return serial_motor_demo_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<serial_motor_demo_msgs::msg::EncoderVals>()
{
  return "serial_motor_demo_msgs::msg::EncoderVals";
}

template<>
inline const char * name<serial_motor_demo_msgs::msg::EncoderVals>()
{
  return "serial_motor_demo_msgs/msg/EncoderVals";
}

template<>
struct has_fixed_size<serial_motor_demo_msgs::msg::EncoderVals>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<serial_motor_demo_msgs::msg::EncoderVals>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<serial_motor_demo_msgs::msg::EncoderVals>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SERIAL_MOTOR_DEMO_MSGS__MSG__DETAIL__ENCODER_VALS__TRAITS_HPP_
