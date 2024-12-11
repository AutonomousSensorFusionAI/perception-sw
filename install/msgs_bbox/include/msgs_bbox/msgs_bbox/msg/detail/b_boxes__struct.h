// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_H_
#define MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bbox'
#include "msgs_bbox/msg/detail/b_box__struct.h"

/// Struct defined in msg/BBoxes in the package msgs_bbox.
typedef struct msgs_bbox__msg__BBoxes
{
  msgs_bbox__msg__BBox__Sequence bbox;
} msgs_bbox__msg__BBoxes;

// Struct for a sequence of msgs_bbox__msg__BBoxes.
typedef struct msgs_bbox__msg__BBoxes__Sequence
{
  msgs_bbox__msg__BBoxes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs_bbox__msg__BBoxes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_H_
