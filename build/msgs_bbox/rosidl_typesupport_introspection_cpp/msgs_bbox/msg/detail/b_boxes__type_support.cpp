// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "msgs_bbox/msg/detail/b_boxes__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace msgs_bbox
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BBoxes_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) msgs_bbox::msg::BBoxes(_init);
}

void BBoxes_fini_function(void * message_memory)
{
  auto typed_message = static_cast<msgs_bbox::msg::BBoxes *>(message_memory);
  typed_message->~BBoxes();
}

size_t size_function__BBoxes__bbox(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<msgs_bbox::msg::BBox> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BBoxes__bbox(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<msgs_bbox::msg::BBox> *>(untyped_member);
  return &member[index];
}

void * get_function__BBoxes__bbox(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<msgs_bbox::msg::BBox> *>(untyped_member);
  return &member[index];
}

void fetch_function__BBoxes__bbox(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const msgs_bbox::msg::BBox *>(
    get_const_function__BBoxes__bbox(untyped_member, index));
  auto & value = *reinterpret_cast<msgs_bbox::msg::BBox *>(untyped_value);
  value = item;
}

void assign_function__BBoxes__bbox(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<msgs_bbox::msg::BBox *>(
    get_function__BBoxes__bbox(untyped_member, index));
  const auto & value = *reinterpret_cast<const msgs_bbox::msg::BBox *>(untyped_value);
  item = value;
}

void resize_function__BBoxes__bbox(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<msgs_bbox::msg::BBox> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BBoxes_message_member_array[1] = {
  {
    "bbox",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<msgs_bbox::msg::BBox>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_bbox::msg::BBoxes, bbox),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBoxes__bbox,  // size() function pointer
    get_const_function__BBoxes__bbox,  // get_const(index) function pointer
    get_function__BBoxes__bbox,  // get(index) function pointer
    fetch_function__BBoxes__bbox,  // fetch(index, &value) function pointer
    assign_function__BBoxes__bbox,  // assign(index, value) function pointer
    resize_function__BBoxes__bbox  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BBoxes_message_members = {
  "msgs_bbox::msg",  // message namespace
  "BBoxes",  // message name
  1,  // number of fields
  sizeof(msgs_bbox::msg::BBoxes),
  BBoxes_message_member_array,  // message members
  BBoxes_init_function,  // function to initialize message memory (memory has to be allocated)
  BBoxes_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BBoxes_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BBoxes_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace msgs_bbox


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<msgs_bbox::msg::BBoxes>()
{
  return &::msgs_bbox::msg::rosidl_typesupport_introspection_cpp::BBoxes_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, msgs_bbox, msg, BBoxes)() {
  return &::msgs_bbox::msg::rosidl_typesupport_introspection_cpp::BBoxes_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
