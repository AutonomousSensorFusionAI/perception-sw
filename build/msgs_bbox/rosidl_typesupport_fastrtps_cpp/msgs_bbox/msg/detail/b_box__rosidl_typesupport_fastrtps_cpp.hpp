// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "msgs_bbox/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "msgs_bbox/msg/detail/b_box__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace msgs_bbox
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
cdr_serialize(
  const msgs_bbox::msg::BBox & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msgs_bbox::msg::BBox & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
get_serialized_size(
  const msgs_bbox::msg::BBox & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
max_serialized_size_BBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msgs_bbox

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msgs_bbox, msg, BBox)();

#ifdef __cplusplus
}
#endif

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
