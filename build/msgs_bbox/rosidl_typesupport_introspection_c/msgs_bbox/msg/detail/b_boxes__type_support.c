// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msgs_bbox/msg/detail/b_boxes__rosidl_typesupport_introspection_c.h"
#include "msgs_bbox/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msgs_bbox/msg/detail/b_boxes__functions.h"
#include "msgs_bbox/msg/detail/b_boxes__struct.h"


// Include directives for member types
// Member `bbox`
#include "msgs_bbox/msg/b_box.h"
// Member `bbox`
#include "msgs_bbox/msg/detail/b_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msgs_bbox__msg__BBoxes__init(message_memory);
}

void msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_fini_function(void * message_memory)
{
  msgs_bbox__msg__BBoxes__fini(message_memory);
}

size_t msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__size_function__BBoxes__bbox(
  const void * untyped_member)
{
  const msgs_bbox__msg__BBox__Sequence * member =
    (const msgs_bbox__msg__BBox__Sequence *)(untyped_member);
  return member->size;
}

const void * msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_const_function__BBoxes__bbox(
  const void * untyped_member, size_t index)
{
  const msgs_bbox__msg__BBox__Sequence * member =
    (const msgs_bbox__msg__BBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_function__BBoxes__bbox(
  void * untyped_member, size_t index)
{
  msgs_bbox__msg__BBox__Sequence * member =
    (msgs_bbox__msg__BBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__fetch_function__BBoxes__bbox(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const msgs_bbox__msg__BBox * item =
    ((const msgs_bbox__msg__BBox *)
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_const_function__BBoxes__bbox(untyped_member, index));
  msgs_bbox__msg__BBox * value =
    (msgs_bbox__msg__BBox *)(untyped_value);
  *value = *item;
}

void msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__assign_function__BBoxes__bbox(
  void * untyped_member, size_t index, const void * untyped_value)
{
  msgs_bbox__msg__BBox * item =
    ((msgs_bbox__msg__BBox *)
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_function__BBoxes__bbox(untyped_member, index));
  const msgs_bbox__msg__BBox * value =
    (const msgs_bbox__msg__BBox *)(untyped_value);
  *item = *value;
}

bool msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__resize_function__BBoxes__bbox(
  void * untyped_member, size_t size)
{
  msgs_bbox__msg__BBox__Sequence * member =
    (msgs_bbox__msg__BBox__Sequence *)(untyped_member);
  msgs_bbox__msg__BBox__Sequence__fini(member);
  return msgs_bbox__msg__BBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_member_array[1] = {
  {
    "bbox",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_bbox__msg__BBoxes, bbox),  // bytes offset in struct
    NULL,  // default value
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__size_function__BBoxes__bbox,  // size() function pointer
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_const_function__BBoxes__bbox,  // get_const(index) function pointer
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__get_function__BBoxes__bbox,  // get(index) function pointer
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__fetch_function__BBoxes__bbox,  // fetch(index, &value) function pointer
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__assign_function__BBoxes__bbox,  // assign(index, value) function pointer
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__resize_function__BBoxes__bbox  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_members = {
  "msgs_bbox__msg",  // message namespace
  "BBoxes",  // message name
  1,  // number of fields
  sizeof(msgs_bbox__msg__BBoxes),
  msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_member_array,  // message members
  msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_init_function,  // function to initialize message memory (memory has to be allocated)
  msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_type_support_handle = {
  0,
  &msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msgs_bbox
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msgs_bbox, msg, BBoxes)() {
  msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msgs_bbox, msg, BBox)();
  if (!msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_type_support_handle.typesupport_identifier) {
    msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &msgs_bbox__msg__BBoxes__rosidl_typesupport_introspection_c__BBoxes_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
