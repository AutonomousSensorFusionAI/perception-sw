// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOX__TRAITS_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs_bbox/msg/detail/b_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msgs_bbox
{

namespace msg
{

inline void to_flow_style_yaml(
  const BBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: cls
  {
    out << "cls: ";
    rosidl_generator_traits::value_to_yaml(msg.cls, out);
    out << ", ";
  }

  // member: x1
  {
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << ", ";
  }

  // member: y1
  {
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << ", ";
  }

  // member: x2
  {
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << ", ";
  }

  // member: y2
  {
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << ", ";
  }

  // member: conf
  {
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cls: ";
    rosidl_generator_traits::value_to_yaml(msg.cls, out);
    out << "\n";
  }

  // member: x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << "\n";
  }

  // member: y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << "\n";
  }

  // member: x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << "\n";
  }

  // member: y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << "\n";
  }

  // member: conf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conf: ";
    rosidl_generator_traits::value_to_yaml(msg.conf, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BBox & msg, bool use_flow_style = false)
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

}  // namespace msgs_bbox

namespace rosidl_generator_traits
{

[[deprecated("use msgs_bbox::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msgs_bbox::msg::BBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs_bbox::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs_bbox::msg::to_yaml() instead")]]
inline std::string to_yaml(const msgs_bbox::msg::BBox & msg)
{
  return msgs_bbox::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msgs_bbox::msg::BBox>()
{
  return "msgs_bbox::msg::BBox";
}

template<>
inline const char * name<msgs_bbox::msg::BBox>()
{
  return "msgs_bbox/msg/BBox";
}

template<>
struct has_fixed_size<msgs_bbox::msg::BBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<msgs_bbox::msg::BBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<msgs_bbox::msg::BBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOX__TRAITS_HPP_
