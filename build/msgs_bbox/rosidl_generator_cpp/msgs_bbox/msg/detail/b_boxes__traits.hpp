// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOXES__TRAITS_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOXES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msgs_bbox/msg/detail/b_boxes__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'bbox'
#include "msgs_bbox/msg/detail/b_box__traits.hpp"

namespace msgs_bbox
{

namespace msg
{

inline void to_flow_style_yaml(
  const BBoxes & msg,
  std::ostream & out)
{
  out << "{";
  // member: bbox
  {
    if (msg.bbox.size() == 0) {
      out << "bbox: []";
    } else {
      out << "bbox: [";
      size_t pending_items = msg.bbox.size();
      for (auto item : msg.bbox) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BBoxes & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bbox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bbox.size() == 0) {
      out << "bbox: []\n";
    } else {
      out << "bbox:\n";
      for (auto item : msg.bbox) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BBoxes & msg, bool use_flow_style = false)
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
  const msgs_bbox::msg::BBoxes & msg,
  std::ostream & out, size_t indentation = 0)
{
  msgs_bbox::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msgs_bbox::msg::to_yaml() instead")]]
inline std::string to_yaml(const msgs_bbox::msg::BBoxes & msg)
{
  return msgs_bbox::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msgs_bbox::msg::BBoxes>()
{
  return "msgs_bbox::msg::BBoxes";
}

template<>
inline const char * name<msgs_bbox::msg::BBoxes>()
{
  return "msgs_bbox/msg/BBoxes";
}

template<>
struct has_fixed_size<msgs_bbox::msg::BBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msgs_bbox::msg::BBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msgs_bbox::msg::BBoxes>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOXES__TRAITS_HPP_
