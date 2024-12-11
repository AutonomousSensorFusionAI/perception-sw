// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice
#include "msgs_bbox/msg/detail/b_boxes__rosidl_typesupport_fastrtps_cpp.hpp"
#include "msgs_bbox/msg/detail/b_boxes__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace msgs_bbox
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const msgs_bbox::msg::BBox &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  msgs_bbox::msg::BBox &);
size_t get_serialized_size(
  const msgs_bbox::msg::BBox &,
  size_t current_alignment);
size_t
max_serialized_size_BBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace msgs_bbox


namespace msgs_bbox
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
cdr_serialize(
  const msgs_bbox::msg::BBoxes & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bbox
  {
    size_t size = ros_message.bbox.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      msgs_bbox::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.bbox[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msgs_bbox::msg::BBoxes & ros_message)
{
  // Member: bbox
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.bbox.resize(size);
    for (size_t i = 0; i < size; i++) {
      msgs_bbox::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.bbox[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
get_serialized_size(
  const msgs_bbox::msg::BBoxes & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bbox
  {
    size_t array_size = ros_message.bbox.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        msgs_bbox::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.bbox[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_bbox
max_serialized_size_BBoxes(
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


  // Member: bbox
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        msgs_bbox::msg::typesupport_fastrtps_cpp::max_serialized_size_BBox(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = msgs_bbox::msg::BBoxes;
    is_plain =
      (
      offsetof(DataType, bbox) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _BBoxes__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const msgs_bbox::msg::BBoxes *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BBoxes__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<msgs_bbox::msg::BBoxes *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BBoxes__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const msgs_bbox::msg::BBoxes *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BBoxes__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_BBoxes(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _BBoxes__callbacks = {
  "msgs_bbox::msg",
  "BBoxes",
  _BBoxes__cdr_serialize,
  _BBoxes__cdr_deserialize,
  _BBoxes__get_serialized_size,
  _BBoxes__max_serialized_size
};

static rosidl_message_type_support_t _BBoxes__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BBoxes__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msgs_bbox

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_msgs_bbox
const rosidl_message_type_support_t *
get_message_type_support_handle<msgs_bbox::msg::BBoxes>()
{
  return &msgs_bbox::msg::typesupport_fastrtps_cpp::_BBoxes__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msgs_bbox, msg, BBoxes)() {
  return &msgs_bbox::msg::typesupport_fastrtps_cpp::_BBoxes__handle;
}

#ifdef __cplusplus
}
#endif
